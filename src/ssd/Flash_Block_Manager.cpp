
#include "../nvm_chip/flash_memory/Physical_Page_Address.h"
#include "../sim/Engine.h"
#include "Flash_Block_Manager.h"
#include "Stats.h"

namespace SSD_Components
{
	Flash_Block_Manager::Flash_Block_Manager(GC_and_WL_Unit_Base* gc_and_wl_unit, unsigned int max_allowed_block_erase_count, unsigned int total_concurrent_streams_no,
		unsigned int channel_count, unsigned int chip_no_per_channel, unsigned int die_no_per_chip, unsigned int plane_no_per_die,
		unsigned int block_no_per_plane, unsigned int page_no_per_block, unsigned int tlc_block_no_per_plane)
		: Flash_Block_Manager_Base(gc_and_wl_unit, max_allowed_block_erase_count, total_concurrent_streams_no, channel_count, chip_no_per_channel, die_no_per_chip,
			plane_no_per_die, block_no_per_plane, page_no_per_block, tlc_block_no_per_plane)
	{
	}

	Flash_Block_Manager::~Flash_Block_Manager()
	{
	}

	void Flash_Block_Manager::Allocate_block_and_page_in_plane_for_user_write(const stream_id_type stream_id, NVM::FlashMemory::Physical_Page_Address& page_address)
	{
		if (slc_cache_enabled) {
			const unsigned long long plane_count = (unsigned long long)channel_count * chip_no_per_channel * die_no_per_chip * plane_no_per_die;
			PlaneBookKeepingType* selected_plane = NULL;
			NVM::FlashMemory::Physical_Page_Address selected_address;
			for (unsigned long long offset = 0; offset < plane_count; offset++) {
				const unsigned long long candidate_index = (slc_next_allocation_unit + offset) % plane_count;
				unsigned long long index = candidate_index;

				// Channel is the fastest-changing dimension, followed by chip,
				// die, and plane. This sends consecutive SLC writes to different
				// parallel units instead of filling one plane at a time.
				selected_address.ChannelID = (flash_channel_ID_type)(index % channel_count);
				index /= channel_count;
				selected_address.ChipID = (flash_chip_ID_type)(index % chip_no_per_channel);
				index /= chip_no_per_channel;
				selected_address.DieID = (flash_die_ID_type)(index % die_no_per_chip);
				index /= die_no_per_chip;
				selected_address.PlaneID = (flash_plane_ID_type)(index % plane_no_per_die);
				PlaneBookKeepingType* candidate = &plane_manager[selected_address.ChannelID][selected_address.ChipID][selected_address.DieID][selected_address.PlaneID];
				Block_Pool_Slot_Type* frontier = candidate->SLC_wf[stream_id];
				if ((frontier != NULL && frontier->Current_page_write_index < frontier->Usable_page_count)
					|| !candidate->Free_slc_block_pool.empty()) {
					selected_plane = candidate;
					slc_next_allocation_unit = (candidate_index + 1) % plane_count;
					break;
				}
			}

			if (selected_plane != NULL) {
				page_address.ChannelID = selected_address.ChannelID;
				page_address.ChipID = selected_address.ChipID;
				page_address.DieID = selected_address.DieID;
				page_address.PlaneID = selected_address.PlaneID;
				if (selected_plane->SLC_wf[stream_id] == NULL) {
					selected_plane->SLC_wf[stream_id] = selected_plane->Get_a_free_slc_block(stream_id);
				}
				Block_Pool_Slot_Type* frontier = selected_plane->SLC_wf[stream_id];
				selected_plane->Valid_pages_count++;
				selected_plane->Free_pages_count--;
				if (gc_and_wl_unit != NULL)
					gc_and_wl_unit->Record_slc_user_write();
				free_slc_pages--;
				page_address.BlockID = frontier->BlockID;
				page_address.PageID = frontier->Current_page_write_index++;
				program_transaction_issued(page_address);
				if (frontier->Current_page_write_index == frontier->Usable_page_count) {
					Register_closed_slc_block(page_address);
					selected_plane->SLC_wf[stream_id] = NULL;
				}
				selected_plane->Check_bookkeeping_correctness(page_address);
				return;
			}
			Enter_slc_bypass();
		}

		Allocate_block_and_page_in_plane_for_tlc_write(stream_id, page_address);
	}

	void Flash_Block_Manager::Allocate_block_and_page_in_plane_for_tlc_write(const stream_id_type stream_id, NVM::FlashMemory::Physical_Page_Address& page_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Valid_pages_count++;
		plane_record->Free_pages_count--;	
		
		if(plane_record->Data_wf[stream_id] == NULL){
			plane_record->Data_wf[stream_id] = plane_record->Get_a_free_block(stream_id, false);
			gc_and_wl_unit->Check_gc_required(plane_record->Get_free_block_pool_size(), page_address);
		}	

		page_address.BlockID = plane_record->Data_wf[stream_id]->BlockID;
		page_address.PageID = plane_record->Data_wf[stream_id]->Current_page_write_index++;
		Record_tlc_page_allocation(page_address);
		program_transaction_issued(page_address);

		//The current write frontier block is written to the end
		if(plane_record->Data_wf[stream_id]->Current_page_write_index == pages_no_per_block) {
			//Assign a new write frontier block
			plane_record->Data_wf[stream_id] = plane_record->Get_a_free_block(stream_id, false);
			gc_and_wl_unit->Check_gc_required(plane_record->Get_free_block_pool_size(), page_address);
		}

		plane_record->Check_bookkeeping_correctness(page_address);
	}

	void Flash_Block_Manager::Allocate_block_and_page_in_plane_for_gc_write(const stream_id_type stream_id, NVM::FlashMemory::Physical_Page_Address& page_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Valid_pages_count++;
		plane_record->Free_pages_count--;	

		if(plane_record->GC_wf[stream_id] == NULL){
			plane_record->GC_wf[stream_id] = plane_record->Get_a_free_block(stream_id, false);
			//gc_and_wl_unit->Check_gc_required(plane_record->Get_free_block_pool_size(), page_address);
		}

		page_address.BlockID = plane_record->GC_wf[stream_id]->BlockID;
		page_address.PageID = plane_record->GC_wf[stream_id]->Current_page_write_index++;
		Record_tlc_page_allocation(page_address);
		program_transaction_issued(page_address);
		
		//The current write frontier block is written to the end
		if (plane_record->GC_wf[stream_id]->Current_page_write_index == pages_no_per_block) {
			//Assign a new write frontier block
			plane_record->GC_wf[stream_id] = plane_record->Get_a_free_block(stream_id, false);
			//gc_and_wl_unit->Check_gc_required(plane_record->Get_free_block_pool_size(), page_address);
		}
		plane_record->Check_bookkeeping_correctness(page_address);
	}
	
	void Flash_Block_Manager::Allocate_Pages_in_block_and_invalidate_remaining_for_preconditioning(const stream_id_type stream_id, const NVM::FlashMemory::Physical_Page_Address& plane_address, std::vector<NVM::FlashMemory::Physical_Page_Address>& page_addresses)
	{
		if(page_addresses.size() > pages_no_per_block) {
			PRINT_ERROR("Error while precondition a physical block: the size of the address list is larger than the pages_no_per_block!")
		}
			
		PlaneBookKeepingType *plane_record = &plane_manager[plane_address.ChannelID][plane_address.ChipID][plane_address.DieID][plane_address.PlaneID];
		if (plane_record->Data_wf[stream_id]->Current_page_write_index > 0) {
			PRINT_ERROR("Illegal operation: the Allocate_Pages_in_block_and_invalidate_remaining_for_preconditioning function should be executed for an erased block!")
		}

		if(plane_record->Data_wf[stream_id] == NULL){
			plane_record->Data_wf[stream_id] = plane_record->Get_a_free_block(stream_id, false);
		}

		//Assign physical addresses
		for (int i = 0; i < page_addresses.size(); i++) {
			plane_record->Valid_pages_count++;
			plane_record->Free_pages_count--;
			page_addresses[i].BlockID = plane_record->Data_wf[stream_id]->BlockID;
			page_addresses[i].PageID = plane_record->Data_wf[stream_id]->Current_page_write_index++;
			plane_record->Check_bookkeeping_correctness(page_addresses[i]);
		}

		//Invalidate the remaining pages in the block
		NVM::FlashMemory::Physical_Page_Address target_address(plane_address);
		while (plane_record->Data_wf[stream_id]->Current_page_write_index < pages_no_per_block) {
			plane_record->Free_pages_count--;
			target_address.BlockID = plane_record->Data_wf[stream_id]->BlockID;
			target_address.PageID = plane_record->Data_wf[stream_id]->Current_page_write_index++;
			Invalidate_page_in_block_for_preconditioning(stream_id, target_address);
			plane_record->Check_bookkeeping_correctness(plane_address);
		}

		//Update the write frontier
		plane_record->Data_wf[stream_id] = plane_record->Get_a_free_block(stream_id, false);
	}

	void Flash_Block_Manager::Allocate_block_and_page_in_plane_for_translation_write(const stream_id_type streamID, NVM::FlashMemory::Physical_Page_Address& page_address, bool is_for_gc)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Valid_pages_count++;
		plane_record->Free_pages_count--;

		if(plane_record->Translation_wf[streamID] == NULL){
			plane_record->Translation_wf[streamID] = plane_record->Get_a_free_block(streamID, true);
			if (!is_for_gc) {
				gc_and_wl_unit->Check_gc_required(plane_record->Get_free_block_pool_size(), page_address);
			}
		}

		page_address.BlockID = plane_record->Translation_wf[streamID]->BlockID;
		page_address.PageID = plane_record->Translation_wf[streamID]->Current_page_write_index++;
		Record_tlc_page_allocation(page_address);
		if(!is_for_gc)
			program_transaction_issued(page_address);

		//The current write frontier block for translation pages is written to the end
		if (plane_record->Translation_wf[streamID]->Current_page_write_index == pages_no_per_block) {
			//Assign a new write frontier block
			plane_record->Translation_wf[streamID] = plane_record->Get_a_free_block(streamID, true);
			if (!is_for_gc) {
				gc_and_wl_unit->Check_gc_required(plane_record->Get_free_block_pool_size(), page_address);
			}
		}
		plane_record->Check_bookkeeping_correctness(page_address);
	}

	void Flash_Block_Manager::Allocate_block_and_page_in_plane_for_RAID_write(const stream_id_type streamID, NVM::FlashMemory::Physical_Page_Address& page_address, bool is_for_gc)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Valid_pages_count++;
		plane_record->Free_pages_count--;

		if(plane_record->Raid_wf == NULL){
			plane_record->Raid_wf = plane_record->Get_a_free_block(streamID, false);
			plane_record->Raid_wf->Holds_RAID_data = true;
			if (!is_for_gc) {
				gc_and_wl_unit->Check_gc_required(plane_record->Get_free_block_pool_size(), page_address);
			}
		}

		page_address.BlockID = plane_record->Raid_wf->BlockID;
		page_address.PageID = plane_record->Raid_wf->Current_page_write_index++;
		Record_tlc_page_allocation(page_address);
		if(!is_for_gc)
			program_transaction_issued(page_address);

		//The current write frontier block for translation pages is written to the end
		if (plane_record->Raid_wf->Current_page_write_index == pages_no_per_block) {
			//Assign a new write frontier block
			plane_record->Raid_wf = plane_record->Get_a_free_block(streamID, false);
			plane_record->Raid_wf->Holds_RAID_data = true;
			if (!is_for_gc) {
				gc_and_wl_unit->Check_gc_required(plane_record->Get_free_block_pool_size(), page_address);
			}
		}
		plane_record->Check_bookkeeping_correctness(page_address);
	}


	inline void Flash_Block_Manager::Invalidate_page_in_block(
		const stream_id_type stream_id,
		const NVM::FlashMemory::Physical_Page_Address& page_address,
		bool is_host_update)
	{
		PlaneBookKeepingType* plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Invalid_pages_count++;
		plane_record->Valid_pages_count--;
		
		if (!plane_record->Blocks[page_address.BlockID].Holds_RAID_data && plane_record->Blocks[page_address.BlockID].Stream_id != stream_id) {
			
			PRINT_ERROR("Inconsistent status in the Invalidate_page_in_block function! The accessed block is not allocated to stream " << stream_id <<" " <<plane_record->Blocks[page_address.BlockID].Stream_id)
		}
		
		plane_record->Blocks[page_address.BlockID].Invalid_page_count++;
		plane_record->Blocks[page_address.BlockID].Invalid_page_bitmap[page_address.PageID / 64] |= ((uint64_t)0x1) << (page_address.PageID % 64);
		if (is_host_update)
			Record_closed_slc_block_host_invalidation(page_address);
	}

	inline void Flash_Block_Manager::Invalidate_page_in_block_for_preconditioning(const stream_id_type stream_id, const NVM::FlashMemory::Physical_Page_Address& page_address)
	{
		PlaneBookKeepingType* plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Invalid_pages_count++;
		if (plane_record->Blocks[page_address.BlockID].Stream_id != stream_id) {
			PRINT_ERROR("Inconsistent status in the Invalidate_page_in_block function! The accessed block is not allocated to stream " << stream_id)
		}
		plane_record->Blocks[page_address.BlockID].Invalid_page_count++;
		plane_record->Blocks[page_address.BlockID].Invalid_page_bitmap[page_address.PageID / 64] |= ((uint64_t)0x1) << (page_address.PageID % 64);
	}

	void Flash_Block_Manager::Add_erased_block_to_pool(const NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID];
		Block_Pool_Slot_Type* block = &(plane_record->Blocks[block_address.BlockID]);
		plane_record->Free_pages_count += block->Invalid_page_count;
		plane_record->Invalid_pages_count -= block->Invalid_page_count;

		Stats::Block_erase_histogram[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID][block->Erase_count]--;
		block->Erase();
		Stats::Block_erase_histogram[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID][block->Erase_count]++;
		if (block->Is_slc) {
			plane_record->Free_slc_block_pool.insert(std::make_pair(block->Erase_count, block));
			free_slc_pages += block->Usable_page_count;
			if (slc_bypass_active) {
				Stats::SLC_bypass_total_time += Simulator->Time() - slc_bypass_start_time;
				slc_bypass_active = false;
			}
		} else {
			plane_record->Add_to_free_block_pool(block, gc_and_wl_unit->Use_dynamic_wearleveling());
		}
		plane_record->Check_bookkeeping_correctness(block_address);
	}

	inline unsigned int Flash_Block_Manager::Get_pool_size(const NVM::FlashMemory::Physical_Page_Address& plane_address)
	{
		return (unsigned int) plane_manager[plane_address.ChannelID][plane_address.ChipID][plane_address.DieID][plane_address.PlaneID].Free_block_pool.size();
	}
}
