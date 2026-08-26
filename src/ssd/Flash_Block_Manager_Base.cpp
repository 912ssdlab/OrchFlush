#include "Flash_Block_Manager.h"
#include "../sim/Engine.h"
#include "Stats.h"
#include <climits>


namespace SSD_Components
{
	namespace
	{
		// Change this value to tune both the fixed global Hot and Cold regions.
		constexpr unsigned int SLC_TEMPERATURE_REGION_PERCENT = 50;
		static_assert(SLC_TEMPERATURE_REGION_PERCENT > 0
			&& SLC_TEMPERATURE_REGION_PERCENT <= 50,
			"SLC temperature region percentage must be in [1, 50]");
	}

	unsigned int Block_Pool_Slot_Type::Page_vector_size = 0;
	Flash_Block_Manager_Base::Flash_Block_Manager_Base(GC_and_WL_Unit_Base* gc_and_wl_unit, unsigned int max_allowed_block_erase_count, unsigned int total_concurrent_streams_no,
		unsigned int channel_count, unsigned int chip_no_per_channel, unsigned int die_no_per_chip, unsigned int plane_no_per_die,
		unsigned int block_no_per_plane, unsigned int page_no_per_block, unsigned int tlc_block_no_per_plane)
		: gc_and_wl_unit(gc_and_wl_unit), max_allowed_block_erase_count(max_allowed_block_erase_count), total_concurrent_streams_no(total_concurrent_streams_no),
		channel_count(channel_count), chip_no_per_channel(chip_no_per_channel), die_no_per_chip(die_no_per_chip), plane_no_per_die(plane_no_per_die),
		block_no_per_plane(block_no_per_plane),
		tlc_block_no_per_plane(tlc_block_no_per_plane == 0 ? block_no_per_plane : tlc_block_no_per_plane),
		pages_no_per_block(page_no_per_block), slc_pages_no_per_block(page_no_per_block / 3),
		slc_cache_enabled(tlc_block_no_per_plane != 0 && tlc_block_no_per_plane < block_no_per_plane),
			slc_bypass_active(false), slc_bypass_start_time(0),
			total_slc_pages((unsigned long long)(block_no_per_plane - this->tlc_block_no_per_plane)
				* slc_pages_no_per_block * channel_count * chip_no_per_channel * die_no_per_chip * plane_no_per_die),
			free_slc_pages(total_slc_pages), slc_next_allocation_unit(0),
			tlc_next_allocation_unit(0),
			tlc_page_allocation_count((unsigned long long)channel_count * chip_no_per_channel
				* die_no_per_chip * plane_no_per_die, 0)
	{
		plane_manager = new PlaneBookKeepingType***[channel_count];
		for (unsigned int channelID = 0; channelID < channel_count; channelID++) {
			plane_manager[channelID] = new PlaneBookKeepingType**[chip_no_per_channel];
			for (unsigned int chipID = 0; chipID < chip_no_per_channel; chipID++) {
				plane_manager[channelID][chipID] = new PlaneBookKeepingType*[die_no_per_chip];
				for (unsigned int dieID = 0; dieID < die_no_per_chip; dieID++) {
					plane_manager[channelID][chipID][dieID] = new PlaneBookKeepingType[plane_no_per_die];

					//Initialize plane book keeping data structure
					for (unsigned int planeID = 0; planeID < plane_no_per_die; planeID++) {
						unsigned int effective_page_count = this->tlc_block_no_per_plane * pages_no_per_block
							+ (block_no_per_plane - this->tlc_block_no_per_plane) * slc_pages_no_per_block;
						plane_manager[channelID][chipID][dieID][planeID].Total_pages_count = effective_page_count;
						plane_manager[channelID][chipID][dieID][planeID].Free_pages_count = effective_page_count;
						plane_manager[channelID][chipID][dieID][planeID].Valid_pages_count = 0;
						plane_manager[channelID][chipID][dieID][planeID].Invalid_pages_count = 0;
						plane_manager[channelID][chipID][dieID][planeID].Ongoing_erase_operations.clear();
						plane_manager[channelID][chipID][dieID][planeID].Blocks = new Block_Pool_Slot_Type[block_no_per_plane];
						
						//Initialize block pool for plane
						for (unsigned int blockID = 0; blockID < block_no_per_plane; blockID++) {
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].BlockID = blockID;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Current_page_write_index = 0;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Current_status = Block_Service_Status::IDLE;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Invalid_page_count = 0;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Semi_page_count = 0;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Erase_count = 0;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Holds_mapping_data = false;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Has_ongoing_gc_wl = false;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Erase_transaction = NULL;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Ongoing_user_program_count = 0;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Ongoing_user_read_count = 0;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Is_slc = blockID >= this->tlc_block_no_per_plane;
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Usable_page_count =
								plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Is_slc ? slc_pages_no_per_block : pages_no_per_block;
							Block_Pool_Slot_Type::Page_vector_size = pages_no_per_block / (sizeof(uint64_t) * 8) + (pages_no_per_block % (sizeof(uint64_t) * 8) == 0 ? 0 : 1);
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Invalid_page_bitmap = new uint64_t[Block_Pool_Slot_Type::Page_vector_size];
							plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Semi_page_bitmap = new uint64_t[Block_Pool_Slot_Type::Page_vector_size];
							for (unsigned int i = 0; i < Block_Pool_Slot_Type::Page_vector_size; i++) {
								plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Invalid_page_bitmap[i] = All_VALID_PAGE;
								plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Semi_page_bitmap[i] = All_VALID_PAGE;
							}
							if (plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID].Is_slc) {
								plane_manager[channelID][chipID][dieID][planeID].Free_slc_block_pool.insert(
									std::make_pair(0U, &plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID]));
							} else {
								plane_manager[channelID][chipID][dieID][planeID].Add_to_free_block_pool(
									&plane_manager[channelID][chipID][dieID][planeID].Blocks[blockID], false);
							}
						}
						plane_manager[channelID][chipID][dieID][planeID].Data_wf = new Block_Pool_Slot_Type*[total_concurrent_streams_no];
						plane_manager[channelID][chipID][dieID][planeID].Translation_wf = new Block_Pool_Slot_Type*[total_concurrent_streams_no];
						plane_manager[channelID][chipID][dieID][planeID].GC_wf = new Block_Pool_Slot_Type*[total_concurrent_streams_no];
						plane_manager[channelID][chipID][dieID][planeID].SLC_wf = new Block_Pool_Slot_Type*[total_concurrent_streams_no];
						for (unsigned int stream_cntr = 0; stream_cntr < total_concurrent_streams_no; stream_cntr++) {
							plane_manager[channelID][chipID][dieID][planeID].Data_wf[stream_cntr] = NULL;
							plane_manager[channelID][chipID][dieID][planeID].Translation_wf[stream_cntr] = NULL;
							plane_manager[channelID][chipID][dieID][planeID].GC_wf[stream_cntr] = NULL;
							plane_manager[channelID][chipID][dieID][planeID].SLC_wf[stream_cntr] = NULL;
						}
						plane_manager[channelID][chipID][dieID][planeID].Raid_wf = NULL;
					}
				}
			}
		}
	}

	Flash_Block_Manager_Base::~Flash_Block_Manager_Base() 
	{
		long long count = 0;
		long long count1 = 0;
		long long count2 = 0;
		long long count3 = 0;
		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++) {
			for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++) {
				for (unsigned int die_id = 0; die_id < die_no_per_chip; die_id++) {
					for (unsigned int plane_id = 0; plane_id < plane_no_per_die; plane_id++) {
						PlaneBookKeepingType *plane_record = &plane_manager[channel_id][chip_id][die_id][plane_id];
						for (unsigned int blockID = 0; blockID < block_no_per_plane; blockID++) {
							count += plane_record->Blocks[blockID].Ongoing_user_program_count;
							count2 += plane_record->Blocks[blockID].Ongoing_user_read_count;
							if(plane_record->Blocks[blockID].Ongoing_user_program_count < 0){
								count1 += plane_record->Blocks[blockID].Ongoing_user_program_count;
							}
							if(plane_record->Blocks[blockID].Ongoing_user_read_count < 0){
								count1 += plane_record->Blocks[blockID].Ongoing_user_read_count;
							}
						}
						
					}
				}
				
			}
			
		}
		std::cout << count << "\t" <<count1 << "\t" <<count2 << "\t" << count3<<std::endl;

		count = 0;
		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++) {
			for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++) {
				for (unsigned int die_id = 0; die_id < die_no_per_chip; die_id++) {
					for (unsigned int plane_id = 0; plane_id < plane_no_per_die; plane_id++) {
						for (unsigned int blockID = 0; blockID < block_no_per_plane; blockID++) {
							if(plane_manager[channel_id][chip_id][die_id][plane_id].Blocks[blockID].Has_ongoing_gc_wl){
								count++;
							}
							delete[] plane_manager[channel_id][chip_id][die_id][plane_id].Blocks[blockID].Invalid_page_bitmap;
							delete[] plane_manager[channel_id][chip_id][die_id][plane_id].Blocks[blockID].Semi_page_bitmap;
						}
						delete[] plane_manager[channel_id][chip_id][die_id][plane_id].Blocks;
						delete[] plane_manager[channel_id][chip_id][die_id][plane_id].GC_wf;
						delete[] plane_manager[channel_id][chip_id][die_id][plane_id].Data_wf;
						delete[] plane_manager[channel_id][chip_id][die_id][plane_id].Translation_wf;
						delete[] plane_manager[channel_id][chip_id][die_id][plane_id].SLC_wf;
					}
					delete[] plane_manager[channel_id][chip_id][die_id];
				}
				delete[] plane_manager[channel_id][chip_id];
			}
			delete[] plane_manager[channel_id];
		}
		delete[] plane_manager;
		std::cout << count <<std::endl;
	}

	void Flash_Block_Manager_Base::Set_GC_and_WL_Unit(GC_and_WL_Unit_Base* gcwl)
	{
		this->gc_and_wl_unit = gcwl;
	}

	void Block_Pool_Slot_Type::Erase()
	{
		Current_page_write_index = 0;
		Invalid_page_count = 0;
		Semi_page_count = 0;
		Erase_count++;
		for (unsigned int i = 0; i < Block_Pool_Slot_Type::Page_vector_size; i++) {
			Invalid_page_bitmap[i] = All_VALID_PAGE;
			if(Semi_page_bitmap[i]){
				throw "if(Semi_page_bitmap[i])";
			}
			Semi_page_bitmap[i] = All_VALID_PAGE;
		}
		Stream_id = NO_STREAM;
		Holds_mapping_data = false;
		Erase_transaction = NULL;
	}

	Block_Pool_Slot_Type* PlaneBookKeepingType::Get_a_free_block(stream_id_type stream_id, bool for_mapping_data)
	{
		Block_Pool_Slot_Type* new_block = NULL;
		if (Free_block_pool.size() == 0) {
			PRINT_ERROR("Requesting a free block from an empty pool!")
		}
		new_block = (*Free_block_pool.begin()).second;//Assign a new write frontier block
		Free_block_pool.erase(Free_block_pool.begin());
		new_block->Stream_id = stream_id;
		
		new_block->Holds_mapping_data = for_mapping_data;
		new_block->Holds_RAID_data = false;
		Block_usage_history.push(new_block->BlockID);

		return new_block;
	}

	Block_Pool_Slot_Type* PlaneBookKeepingType::Get_a_free_slc_block(stream_id_type stream_id)
	{
		if (Free_slc_block_pool.empty()) {
			return NULL;
		}
		Block_Pool_Slot_Type* new_block = Free_slc_block_pool.begin()->second;
		Free_slc_block_pool.erase(Free_slc_block_pool.begin());
		new_block->Stream_id = stream_id;
		new_block->Holds_mapping_data = false;
		new_block->Holds_RAID_data = false;
		return new_block;
	}
	
	void PlaneBookKeepingType::Check_bookkeeping_correctness(const NVM::FlashMemory::Physical_Page_Address& plane_address)
	{
		if (Total_pages_count != Free_pages_count + Valid_pages_count + Invalid_pages_count) {
			PRINT_ERROR("Inconsistent status in the plane bookkeeping record!")
		}
		if (Free_pages_count == 0) {
			PRINT_ERROR("Plane " << "@" << plane_address.ChannelID << "@" << plane_address.ChipID << "@" << plane_address.DieID << "@" << plane_address.PlaneID << " pool size: " << Get_free_block_pool_size() << " ran out of free pages! Bad resource management! It is not safe to continue simulation!");
		}
	}

	unsigned int PlaneBookKeepingType::Get_free_block_pool_size()
	{
		return (unsigned int)Free_block_pool.size();
	}

	void PlaneBookKeepingType::Add_to_free_block_pool(Block_Pool_Slot_Type* block, bool consider_dynamic_wl)
	{
		if (consider_dynamic_wl) {
			std::pair<unsigned int, Block_Pool_Slot_Type*> entry(block->Erase_count, block);
			Free_block_pool.insert(entry);
		} else {
			std::pair<unsigned int, Block_Pool_Slot_Type*> entry(0, block);
			Free_block_pool.insert(entry);
		}
	}

	bool Flash_Block_Manager_Base::Is_slc_block(const NVM::FlashMemory::Physical_Page_Address& address) const
	{
		return slc_cache_enabled && address.BlockID >= tlc_block_no_per_plane;
	}

	bool Flash_Block_Manager_Base::Has_writable_slc_page(const stream_id_type stream_id) const
	{
		if (!slc_cache_enabled)
			return false;

		for (unsigned int channel = 0; channel < channel_count; channel++) {
			for (unsigned int chip = 0; chip < chip_no_per_channel; chip++) {
				for (unsigned int die = 0; die < die_no_per_chip; die++) {
					for (unsigned int plane = 0; plane < plane_no_per_die; plane++) {
						const PlaneBookKeepingType& candidate = plane_manager[channel][chip][die][plane];
						Block_Pool_Slot_Type* frontier = candidate.SLC_wf[stream_id];
						if ((frontier != NULL && frontier->Current_page_write_index < frontier->Usable_page_count)
							|| !candidate.Free_slc_block_pool.empty()) {
							return true;
						}
					}
				}
			}
		}
		return false;
	}

	bool Flash_Block_Manager_Base::Plane_has_writable_tlc_page(
		const PlaneBookKeepingType& plane_record, const stream_id_type stream_id,
		bool for_gc) const
	{
		Block_Pool_Slot_Type* frontier = for_gc
			? plane_record.GC_wf[stream_id]
			: plane_record.Data_wf[stream_id];
		if (frontier == NULL) {
			return !plane_record.Free_block_pool.empty();
		}

		if (frontier->Current_page_write_index >= frontier->Usable_page_count) {
			return false;
		}

		// The allocator immediately opens the next frontier after consuming the
		// last page of the current one, so that allocation also needs a free block.
		return frontier->Current_page_write_index + 1 < frontier->Usable_page_count
			|| !plane_record.Free_block_pool.empty();
	}

	unsigned long long Flash_Block_Manager_Base::Get_plane_linear_index(
		const NVM::FlashMemory::Physical_Page_Address& plane_address) const
	{
		return plane_address.ChannelID
			+ (unsigned long long)channel_count * (plane_address.ChipID
			+ (unsigned long long)chip_no_per_channel * (plane_address.DieID
			+ (unsigned long long)die_no_per_chip * plane_address.PlaneID));
	}

	bool Flash_Block_Manager_Base::Select_global_tlc_plane(
		const stream_id_type stream_id, bool for_gc,
		NVM::FlashMemory::Physical_Page_Address& plane_address,
		bool enforce_user_admission)
	{
		if (stream_id >= total_concurrent_streams_no) {
			return false;
		}

		const unsigned long long plane_count = tlc_page_allocation_count.size();
		unsigned long long selected_index = plane_count;
		unsigned long long selected_write_count = ULLONG_MAX;
		NVM::FlashMemory::Physical_Page_Address candidate_address;

		for (unsigned long long offset = 0; offset < plane_count; offset++) {
			const unsigned long long candidate_index =
				(tlc_next_allocation_unit + offset) % plane_count;
			unsigned long long index = candidate_index;
			candidate_address.ChannelID = (flash_channel_ID_type)(index % channel_count);
			index /= channel_count;
			candidate_address.ChipID = (flash_chip_ID_type)(index % chip_no_per_channel);
			index /= chip_no_per_channel;
			candidate_address.DieID = (flash_die_ID_type)(index % die_no_per_chip);
			index /= die_no_per_chip;
			candidate_address.PlaneID = (flash_plane_ID_type)(index % plane_no_per_die);

			const PlaneBookKeepingType& candidate = plane_manager
				[candidate_address.ChannelID][candidate_address.ChipID]
				[candidate_address.DieID][candidate_address.PlaneID];
			if (!Plane_has_writable_tlc_page(candidate, stream_id, for_gc)) {
				continue;
			}
			if (!for_gc && enforce_user_admission
				&& gc_and_wl_unit->Stop_servicing_tlc_writes(candidate_address)) {
				continue;
			}

			if (tlc_page_allocation_count[candidate_index] < selected_write_count) {
				selected_index = candidate_index;
				selected_write_count = tlc_page_allocation_count[candidate_index];
				plane_address = candidate_address;
			}
		}

		return selected_index != plane_count;
	}

	void Flash_Block_Manager_Base::Record_tlc_page_allocation(
		const NVM::FlashMemory::Physical_Page_Address& page_address)
	{
		if (Is_slc_block(page_address)) {
			return;
		}
		const unsigned long long index = Get_plane_linear_index(page_address);
		tlc_page_allocation_count[index]++;
		tlc_next_allocation_unit = (index + 1) % tlc_page_allocation_count.size();
	}

	void Flash_Block_Manager_Base::Reset_tlc_page_allocation_count()
	{
		for (auto& count : tlc_page_allocation_count) {
			count = 0;
		}
		tlc_next_allocation_unit = 0;
	}

	void Flash_Block_Manager_Base::Enter_slc_bypass()
	{
		if (!slc_cache_enabled)
			return;

		if (!slc_bypass_active) {
			slc_bypass_active = true;
			slc_bypass_start_time = Simulator->Time();
			Stats::SLC_bypass_activations++;
		}
		if (gc_and_wl_unit != NULL) {
			gc_and_wl_unit->Check_slc_gc_required();
		}
	}

	unsigned long long Flash_Block_Manager_Base::Get_total_free_tlc_blocks() const
	{
		unsigned long long count = 0;
		for (unsigned int channel = 0; channel < channel_count; channel++)
			for (unsigned int chip = 0; chip < chip_no_per_channel; chip++)
				for (unsigned int die = 0; die < die_no_per_chip; die++)
					for (unsigned int plane = 0; plane < plane_no_per_die; plane++)
						count += plane_manager[channel][chip][die][plane].Free_block_pool.size();
		return count;
	}

	unsigned long long Flash_Block_Manager_Base::Get_total_free_slc_blocks() const
	{
		unsigned long long count = 0;
		for (unsigned int channel = 0; channel < channel_count; channel++)
			for (unsigned int chip = 0; chip < chip_no_per_channel; chip++)
				for (unsigned int die = 0; die < die_no_per_chip; die++)
					for (unsigned int plane = 0; plane < plane_no_per_die; plane++)
						count += plane_manager[channel][chip][die][plane].Free_slc_block_pool.size();
		return count;
	}

	bool Flash_Block_Manager_Base::Is_slc_free_space_below_percent(
		double threshold_percent) const
	{
		if (!slc_cache_enabled || total_slc_pages == 0)
			return false;
		return (long double)free_slc_pages * 100.0L
			< (long double)total_slc_pages * threshold_percent;
	}

	sim_time_type Flash_Block_Manager_Base::Get_slc_bypass_time() const
	{
		return Stats::SLC_bypass_total_time
			+ (slc_bypass_active ? Simulator->Time() - slc_bypass_start_time : 0);
	}

	void Flash_Block_Manager_Base::Register_closed_slc_block(
		const NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		NVM::FlashMemory::Physical_Page_Address fifo_entry = block_address;
		fifo_entry.PageID = 0;
		const unsigned long long tracking_key =
			Get_slc_block_tracking_key(fifo_entry);
		if (closed_slc_block_tracking.find(tracking_key)
			!= closed_slc_block_tracking.end())
			PRINT_ERROR("A closed SLC block is already present in the FIFO/LRU tracking set")

		// Appending to the final partition and then rebalancing preserves FIFO
		// order while only moving boundary blocks between partitions.
		closed_slc_block_fifo_back.push_back(fifo_entry);
		closed_slc_block_lru_front.push_front(fifo_entry);
		ClosedSlcBlockTrackingEntry tracking_entry;
		tracking_entry.fifo_position = --closed_slc_block_fifo_back.end();
		tracking_entry.lru_position = closed_slc_block_lru_front.begin();
		tracking_entry.fifo_temperature_position = SlcTemperaturePosition::BACK;
		tracking_entry.lru_temperature_position = SlcTemperaturePosition::FRONT;
		tracking_entry.candidate_class = SlcGcCandidateClass::HOT;
		tracking_entry.fifo_sequence = next_closed_slc_block_fifo_sequence++;
		closed_slc_block_tracking.emplace(tracking_key, tracking_entry);
		Refresh_slc_gc_candidate(tracking_key);
		Rebalance_slc_ordering(closed_slc_block_fifo_front,
			closed_slc_block_fifo_overlap, closed_slc_block_fifo_middle,
			closed_slc_block_fifo_back, true);
		Rebalance_slc_ordering(closed_slc_block_lru_front,
			closed_slc_block_lru_overlap, closed_slc_block_lru_middle,
			closed_slc_block_lru_back, false);
	}

	bool Flash_Block_Manager_Base::Dequeue_oldest_closed_slc_block(
		NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		const SlcBlockAddressList* oldest_fifo_list = NULL;
		if (!closed_slc_block_fifo_front.empty())
			oldest_fifo_list = &closed_slc_block_fifo_front;
		else if (!closed_slc_block_fifo_overlap.empty())
			oldest_fifo_list = &closed_slc_block_fifo_overlap;
		else if (!closed_slc_block_fifo_middle.empty())
			oldest_fifo_list = &closed_slc_block_fifo_middle;
		else if (!closed_slc_block_fifo_back.empty())
			oldest_fifo_list = &closed_slc_block_fifo_back;
		if (oldest_fifo_list == NULL)
			return false;

		return Dequeue_tracked_closed_slc_block(Get_slc_block_tracking_key(
			oldest_fifo_list->front()), block_address);
	}

	bool Flash_Block_Manager_Base::Dequeue_idle_slc_block(
		bool occupancy_threshold_reached,
		NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		const size_t block_count = closed_slc_block_tracking.size();
		if (block_count == 0)
			return false;
		if (!closed_slc_cold_candidates.empty())
			return Dequeue_tracked_closed_slc_block(
				closed_slc_cold_candidates.begin()->second, block_address);
		if (occupancy_threshold_reached && !closed_slc_warm_candidates.empty())
			return Dequeue_tracked_closed_slc_block(
				closed_slc_warm_candidates.begin()->second, block_address);
		return false;
	}

	void Flash_Block_Manager_Base::Record_closed_slc_block_host_invalidation(
		const NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		if (!Is_slc_block(block_address))
			return;
		const auto tracking_entry = closed_slc_block_tracking.find(
			Get_slc_block_tracking_key(block_address));
		if (tracking_entry == closed_slc_block_tracking.end())
			return;

		SlcBlockAddressList& old_lru_list = Get_slc_ordering_list(
			tracking_entry->second.lru_temperature_position, false);
		closed_slc_block_lru_front.splice(closed_slc_block_lru_front.begin(),
			old_lru_list, tracking_entry->second.lru_position);
		Set_slc_temperature_position(tracking_entry->first,
			SlcTemperaturePosition::FRONT, false);
		Rebalance_slc_ordering(closed_slc_block_lru_front,
			closed_slc_block_lru_overlap, closed_slc_block_lru_middle,
			closed_slc_block_lru_back, false);
	}

	unsigned long long Flash_Block_Manager_Base::Get_slc_block_tracking_key(
		const NVM::FlashMemory::Physical_Page_Address& block_address) const
	{
		unsigned long long key = block_address.ChannelID;
		key = key * chip_no_per_channel + block_address.ChipID;
		key = key * die_no_per_chip + block_address.DieID;
		key = key * plane_no_per_die + block_address.PlaneID;
		return key * block_no_per_plane + block_address.BlockID;
	}

	size_t Flash_Block_Manager_Base::Get_slc_temperature_region_size() const
	{
		const size_t maximum_block_count = (size_t)Get_slc_block_count()
			* channel_count * chip_no_per_channel * die_no_per_chip
			* plane_no_per_die;
		const size_t maximum_block_hundreds = maximum_block_count / 100;
		const size_t maximum_block_remainder = maximum_block_count % 100;
		return maximum_block_hundreds * SLC_TEMPERATURE_REGION_PERCENT
			+ (maximum_block_remainder * SLC_TEMPERATURE_REGION_PERCENT + 99)
				/ 100;
	}

	size_t Flash_Block_Manager_Base::Get_slc_front_only_region_size() const
	{
		const size_t maximum_block_count = (size_t)Get_slc_block_count()
			* channel_count * chip_no_per_channel * die_no_per_chip
			* plane_no_per_die;
		const size_t temperature_region_size = Get_slc_temperature_region_size();
		return std::min(temperature_region_size,
			maximum_block_count - temperature_region_size);
	}

	size_t Flash_Block_Manager_Base::Get_slc_overlap_region_size() const
	{
		const size_t maximum_block_count = (size_t)Get_slc_block_count()
			* channel_count * chip_no_per_channel * die_no_per_chip
			* plane_no_per_die;
		const size_t temperature_region_size = Get_slc_temperature_region_size();
		return temperature_region_size > maximum_block_count - temperature_region_size
			? 2 * temperature_region_size - maximum_block_count : 0;
	}

	size_t Flash_Block_Manager_Base::Get_slc_middle_region_size() const
	{
		const size_t maximum_block_count = (size_t)Get_slc_block_count()
			* channel_count * chip_no_per_channel * die_no_per_chip
			* plane_no_per_die;
		const size_t temperature_region_size = Get_slc_temperature_region_size();
		return maximum_block_count > 2 * temperature_region_size
			? maximum_block_count - 2 * temperature_region_size : 0;
	}

	Flash_Block_Manager_Base::SlcBlockAddressList&
		Flash_Block_Manager_Base::Get_slc_ordering_list(
		SlcTemperaturePosition position, bool is_fifo_ordering)
	{
		if (is_fifo_ordering) {
			switch (position) {
			case SlcTemperaturePosition::FRONT: return closed_slc_block_fifo_front;
			case SlcTemperaturePosition::OVERLAP: return closed_slc_block_fifo_overlap;
			case SlcTemperaturePosition::MIDDLE: return closed_slc_block_fifo_middle;
			case SlcTemperaturePosition::BACK: return closed_slc_block_fifo_back;
			}
		}
		switch (position) {
		case SlcTemperaturePosition::FRONT: return closed_slc_block_lru_front;
		case SlcTemperaturePosition::OVERLAP: return closed_slc_block_lru_overlap;
		case SlcTemperaturePosition::MIDDLE: return closed_slc_block_lru_middle;
		case SlcTemperaturePosition::BACK: return closed_slc_block_lru_back;
		}
		PRINT_ERROR("Invalid SLC temperature position")
		return closed_slc_block_fifo_front;
	}

	const Flash_Block_Manager_Base::SlcBlockAddressList&
		Flash_Block_Manager_Base::Get_slc_ordering_list(
		SlcTemperaturePosition position, bool is_fifo_ordering) const
	{
		return const_cast<Flash_Block_Manager_Base*>(this)->Get_slc_ordering_list(
			position, is_fifo_ordering);
	}

	void Flash_Block_Manager_Base::Set_slc_temperature_position(
		unsigned long long tracking_key, SlcTemperaturePosition position,
		bool is_fifo_ordering)
	{
		auto tracking_entry = closed_slc_block_tracking.find(tracking_key);
		if (tracking_entry == closed_slc_block_tracking.end())
			PRINT_ERROR("Missing SLC block tracking entry")
		if (is_fifo_ordering)
			tracking_entry->second.fifo_temperature_position = position;
		else
			tracking_entry->second.lru_temperature_position = position;
		Refresh_slc_gc_candidate(tracking_key);
	}

	void Flash_Block_Manager_Base::Refresh_slc_gc_candidate(
		unsigned long long tracking_key)
	{
		auto tracking_entry = closed_slc_block_tracking.find(tracking_key);
		if (tracking_entry == closed_slc_block_tracking.end())
			PRINT_ERROR("Missing SLC block tracking entry")
		ClosedSlcBlockTrackingEntry& entry = tracking_entry->second;
		if (entry.candidate_class == SlcGcCandidateClass::WARM)
			closed_slc_warm_candidates.erase(entry.fifo_sequence);
		else if (entry.candidate_class == SlcGcCandidateClass::COLD)
			closed_slc_cold_candidates.erase(entry.fifo_sequence);

		const bool fifo_is_front = entry.fifo_temperature_position
			== SlcTemperaturePosition::FRONT
			|| entry.fifo_temperature_position == SlcTemperaturePosition::OVERLAP;
		const bool lru_is_front = entry.lru_temperature_position
			== SlcTemperaturePosition::FRONT
			|| entry.lru_temperature_position == SlcTemperaturePosition::OVERLAP;
		const bool fifo_is_back = entry.fifo_temperature_position
			== SlcTemperaturePosition::BACK
			|| entry.fifo_temperature_position == SlcTemperaturePosition::OVERLAP;
		const bool lru_is_back = entry.lru_temperature_position
			== SlcTemperaturePosition::BACK
			|| entry.lru_temperature_position == SlcTemperaturePosition::OVERLAP;
		if (fifo_is_front && lru_is_front)
			entry.candidate_class = SlcGcCandidateClass::HOT;
		else if (fifo_is_back && lru_is_back)
			entry.candidate_class = SlcGcCandidateClass::COLD;
		else
			entry.candidate_class = SlcGcCandidateClass::WARM;

		if (entry.candidate_class == SlcGcCandidateClass::WARM)
			closed_slc_warm_candidates.emplace(entry.fifo_sequence, tracking_key);
		else if (entry.candidate_class == SlcGcCandidateClass::COLD)
			closed_slc_cold_candidates.emplace(entry.fifo_sequence, tracking_key);
	}

	void Flash_Block_Manager_Base::Rebalance_slc_ordering(
		SlcBlockAddressList& front, SlcBlockAddressList& overlap,
		SlcBlockAddressList& middle, SlcBlockAddressList& back,
		bool is_fifo_ordering)
	{
		const size_t total_count = front.size() + overlap.size() + middle.size()
			+ back.size();
		SlcBlockAddressList* lists[] = { &front, &overlap, &middle, &back };
		const SlcTemperaturePosition positions[] = {
			SlcTemperaturePosition::FRONT, SlcTemperaturePosition::OVERLAP,
			SlcTemperaturePosition::MIDDLE, SlcTemperaturePosition::BACK };
		const size_t capacities[] = { Get_slc_front_only_region_size(),
			Get_slc_overlap_region_size(), Get_slc_middle_region_size() };
		size_t remaining = total_count;
		for (size_t index = 0; index < 3; index++) {
			const size_t target = std::min(remaining, capacities[index]);
			while (lists[index]->size() < target) {
				size_t source_index = index + 1;
				while (source_index < 4 && lists[source_index]->empty())
					source_index++;
				if (source_index == 4)
					PRINT_ERROR("Inconsistent SLC ordering partition sizes")
				auto entry = lists[source_index]->begin();
				const unsigned long long key = Get_slc_block_tracking_key(*entry);
				lists[index]->splice(lists[index]->end(), *lists[source_index], entry);
				Set_slc_temperature_position(key, positions[index], is_fifo_ordering);
			}
			while (lists[index]->size() > target) {
				auto entry = --lists[index]->end();
				const unsigned long long key = Get_slc_block_tracking_key(*entry);
				lists[index + 1]->splice(lists[index + 1]->begin(), *lists[index], entry);
				Set_slc_temperature_position(key, positions[index + 1],
					is_fifo_ordering);
			}
			remaining -= target;
		}
	}

	bool Flash_Block_Manager_Base::Dequeue_tracked_closed_slc_block(
		unsigned long long tracking_key,
		NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		auto tracking_entry = closed_slc_block_tracking.find(tracking_key);
		if (tracking_entry == closed_slc_block_tracking.end())
			return false;
		block_address = *tracking_entry->second.fifo_position;
		ClosedSlcBlockTrackingEntry entry = tracking_entry->second;
		if (entry.candidate_class == SlcGcCandidateClass::WARM)
			closed_slc_warm_candidates.erase(entry.fifo_sequence);
		else if (entry.candidate_class == SlcGcCandidateClass::COLD)
			closed_slc_cold_candidates.erase(entry.fifo_sequence);
		Get_slc_ordering_list(entry.lru_temperature_position, false).erase(
			entry.lru_position);
		Get_slc_ordering_list(entry.fifo_temperature_position, true).erase(
			entry.fifo_position);
		closed_slc_block_tracking.erase(tracking_entry);
		Rebalance_slc_ordering(closed_slc_block_fifo_front,
			closed_slc_block_fifo_overlap, closed_slc_block_fifo_middle,
			closed_slc_block_fifo_back, true);
		Rebalance_slc_ordering(closed_slc_block_lru_front,
			closed_slc_block_lru_overlap, closed_slc_block_lru_middle,
			closed_slc_block_lru_back, false);
		return true;
	}

	unsigned int Flash_Block_Manager_Base::Get_min_max_erase_difference(const NVM::FlashMemory::Physical_Page_Address& plane_address)
	{
		unsigned int min_erased_block = 0;
		unsigned int max_erased_block = 0;
		PlaneBookKeepingType *plane_record = &plane_manager[plane_address.ChannelID][plane_address.ChipID][plane_address.DieID][plane_address.PlaneID];
		//throw "enter";
		for (unsigned int i = 1; i < block_no_per_plane; i++) {
			if (plane_record->Blocks[i].Erase_count > plane_record->Blocks[max_erased_block].Erase_count) {
				max_erased_block = i;
			}
			if (plane_record->Blocks[i].Erase_count < plane_record->Blocks[min_erased_block].Erase_count) {
				min_erased_block = i;
			}
		}

		return  plane_record->Blocks[max_erased_block].Erase_count - plane_record->Blocks[min_erased_block].Erase_count;
	}

	flash_block_ID_type Flash_Block_Manager_Base::Get_coldest_block_id(const NVM::FlashMemory::Physical_Page_Address& plane_address)
	{
		unsigned int min_erased_block = 0;
		PlaneBookKeepingType *plane_record = &plane_manager[plane_address.ChannelID][plane_address.ChipID][plane_address.DieID][plane_address.PlaneID];

		for (unsigned int i = 1; i < block_no_per_plane; i++) {
			if (plane_record->Blocks[i].Erase_count < plane_record->Blocks[min_erased_block].Erase_count) {
				min_erased_block = i;
			}
		}
		
		return min_erased_block;
	}

	PlaneBookKeepingType* Flash_Block_Manager_Base::Get_plane_bookkeeping_entry(const NVM::FlashMemory::Physical_Page_Address& plane_address)
	{
		return &(plane_manager[plane_address.ChannelID][plane_address.ChipID][plane_address.DieID][plane_address.PlaneID]);
	}

	bool Flash_Block_Manager_Base::Block_has_ongoing_gc_wl(const NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID];
		return plane_record->Blocks[block_address.BlockID].Has_ongoing_gc_wl;
	}
	
	bool Flash_Block_Manager_Base::has_Ongoing_user_program(const NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID];
		return (plane_record->Blocks[block_address.BlockID].Ongoing_user_program_count == 0);
	}

	bool Flash_Block_Manager_Base::Can_execute_gc_wl(const NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID];
		if(plane_record->Blocks[block_address.BlockID].Ongoing_user_program_count < 0){
			std::cout <<"1 " <<plane_record->Blocks[block_address.BlockID].Ongoing_user_program_count << std::endl;
		}
		if(plane_record->Blocks[block_address.BlockID].Ongoing_user_read_count < 0){
			std::cout <<"2 " << plane_record->Blocks[block_address.BlockID].Ongoing_user_read_count << std::endl;
		}

		if(plane_record->Blocks[block_address.BlockID].Erase_transaction){
			return false;
		}

		return (plane_record->Blocks[block_address.BlockID].Ongoing_user_program_count + plane_record->Blocks[block_address.BlockID].Ongoing_user_read_count == 0);
	}
	
	bool Flash_Block_Manager_Base::Plane_has_ongoing_gc_wl(NVM::FlashMemory::Physical_Page_Address block_address){
		for(int block = 0; block < block_no_per_plane; block++){
			block_address.BlockID = block;
			if(Block_has_ongoing_gc_wl(block_address))
				return true;
		}
		return false;
	}

	void Flash_Block_Manager_Base::GC_WL_started(const NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID];
		plane_record->Blocks[block_address.BlockID].Has_ongoing_gc_wl = true;
		/*if(block_address.ChannelID == 1 && block_address.ChipID == 0 && block_address.DieID == 0 && block_address.PlaneID == 0 && block_address.BlockID == 0){
			std::cout << "111 SET " << std::endl;
			throw "this";
		}*/
	}
	
	void Flash_Block_Manager_Base::program_transaction_issued(const NVM::FlashMemory::Physical_Page_Address& page_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Blocks[page_address.BlockID].Ongoing_user_program_count++;
	}
	
	void Flash_Block_Manager_Base::Read_transaction_issued(const NVM::FlashMemory::Physical_Page_Address& page_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Blocks[page_address.BlockID].Ongoing_user_read_count++;
	}

	void Flash_Block_Manager_Base::Program_transaction_serviced(const NVM::FlashMemory::Physical_Page_Address& page_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Blocks[page_address.BlockID].Ongoing_user_program_count--;
		if(plane_record->Blocks[page_address.BlockID].Ongoing_user_program_count < 0){
			throw "plane_record->Blocks[page_address.BlockID].Ongoing_user_program_count < 0";
		}
	}

	void Flash_Block_Manager_Base::Read_transaction_serviced(const NVM::FlashMemory::Physical_Page_Address& page_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		plane_record->Blocks[page_address.BlockID].Ongoing_user_read_count--;
	}
	
	bool Flash_Block_Manager_Base::Is_having_ongoing_program(const NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID];
		return plane_record->Blocks[block_address.BlockID].Ongoing_user_program_count > 0;
	}

	void Flash_Block_Manager_Base::GC_WL_finished(const NVM::FlashMemory::Physical_Page_Address& block_address)
	{
		PlaneBookKeepingType *plane_record = &plane_manager[block_address.ChannelID][block_address.ChipID][block_address.DieID][block_address.PlaneID];
		plane_record->Blocks[block_address.BlockID].Has_ongoing_gc_wl = false;
	}

	void Flash_Block_Manager_Base::Insemilate_page_in_block(const stream_id_type stream_id, const NVM::FlashMemory::Physical_Page_Address& page_address){
		PlaneBookKeepingType* plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		if(plane_record->Blocks[page_address.BlockID].Semi_page_bitmap[page_address.PageID / 64] & (((uint64_t)0x1) << (page_address.PageID % 64))){
			plane_record->Blocks[page_address.BlockID].Semi_page_bitmap[page_address.PageID / 64] &= (~(((uint64_t)0x1) << (page_address.PageID % 64)));
			// std::cout << (~(((uint64_t)0x1) << (page_address.PageID % 64))) << std::endl;
			// std::cout << "Insemilate_page_in_block" << std::endl;
			plane_record->Blocks[page_address.BlockID].Semi_page_count--;
			if (!plane_record->Blocks[page_address.BlockID].Holds_RAID_data && plane_record->Blocks[page_address.BlockID].Stream_id != stream_id) {
				PRINT_ERROR("Inconsistent status in the Insemilate_page_in_block function! The accessed block is not allocated to stream " << stream_id <<" " <<plane_record->Blocks[page_address.BlockID].Stream_id)
			}
			if(plane_record->Blocks[page_address.BlockID].Semi_page_bitmap[page_address.PageID / 64] & (((uint64_t)0x1) << (page_address.PageID % 64))){
				throw "Insemilate_page_in_block fail!!";
			}
		}else{
			throw "plane_record->Blocks[page_address.BlockID].Semi_page_bitmap[page_address.PageID / 64] & ((uint64_t)0x1) << (page_address.PageID % 64";
		}
	}
	void Flash_Block_Manager_Base::Semilate_page_in_block(const stream_id_type stream_id, const NVM::FlashMemory::Physical_Page_Address& page_address){
		PlaneBookKeepingType* plane_record = &plane_manager[page_address.ChannelID][page_address.ChipID][page_address.DieID][page_address.PlaneID];
		if (!plane_record->Blocks[page_address.BlockID].Holds_RAID_data && plane_record->Blocks[page_address.BlockID].Stream_id != stream_id) {
			PRINT_ERROR("Inconsistent status in the Semilate_page_in_block function! The accessed block is not allocated to stream " << stream_id <<" " <<plane_record->Blocks[page_address.BlockID].Stream_id)
		}
		plane_record->Blocks[page_address.BlockID].Semi_page_count++;
		plane_record->Blocks[page_address.BlockID].Semi_page_bitmap[page_address.PageID / 64] |= (((uint64_t)0x1) << (page_address.PageID % 64));
	}

	bool Flash_Block_Manager_Base::Is_page_Semil(Block_Pool_Slot_Type* block, flash_page_ID_type page_id){
		if ((block->Semi_page_bitmap[page_id / 64] & (((uint64_t)1) << (page_id % 64))) == 0) {
			return false;
		}
		return true;
	}

	bool Flash_Block_Manager_Base::Is_page_valid(Block_Pool_Slot_Type* block, flash_page_ID_type page_id)
	{
		if ((block->Invalid_page_bitmap[page_id / 64] & (((uint64_t)1) << (page_id % 64))) == 0) {
			return true;
		}
		return false;
	}
}
