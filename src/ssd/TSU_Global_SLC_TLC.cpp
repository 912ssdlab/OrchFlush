#include "TSU_Global_SLC_TLC.h"
#include "Host_Interface_Defs.h"
#include "Stats.h"

namespace SSD_Components
{
	TSU_Global_SLC_TLC::TSU_Global_SLC_TLC(const sim_object_id_type& id, FTL* ftl,
		NVM_PHY_ONFI_NVDDR2* controller, unsigned int channel_count,
		unsigned int chip_no_per_channel, unsigned int die_no_per_chip,
		unsigned int plane_no_per_die,
		sim_time_type write_suspend_for_read,
		sim_time_type erase_suspend_for_read,
		sim_time_type erase_suspend_for_write,
		bool erase_suspension_enabled, bool program_suspension_enabled)
		: TSU_Priority_OutOfOrder(id, ftl, controller, channel_count,
			chip_no_per_channel, die_no_per_chip, plane_no_per_die,
			write_suspend_for_read, erase_suspend_for_read,
			erase_suspend_for_write, erase_suspension_enabled,
			program_suspension_enabled, Flash_Scheduling_Type::GLOBAL_SLC_TLC)
		{
			TLCUserWriteTRQueue = new Flash_Transaction_Queue**[channel_count];
			SLCGCReadTRQueue = new Flash_Transaction_Queue*[channel_count];
			SLCGCWriteTRQueue = new Flash_Transaction_Queue*[channel_count];
			SLCGCEraseTRQueue = new Flash_Transaction_Queue*[channel_count];
			for (unsigned int channel = 0; channel < channel_count; channel++) {
				TLCUserWriteTRQueue[channel] = new Flash_Transaction_Queue*[chip_no_per_channel];
				SLCGCReadTRQueue[channel] = new Flash_Transaction_Queue[chip_no_per_channel];
				SLCGCWriteTRQueue[channel] = new Flash_Transaction_Queue[chip_no_per_channel];
				SLCGCEraseTRQueue[channel] = new Flash_Transaction_Queue[chip_no_per_channel];
				for (unsigned int chip = 0; chip < chip_no_per_channel; chip++) {
					TLCUserWriteTRQueue[channel][chip] =
						new Flash_Transaction_Queue[IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS];
					for (unsigned int priority = 0;
						priority < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; priority++) {
						TLCUserWriteTRQueue[channel][chip][priority].Set_id(
							"TLC_User_Write_TR_Queue@" + std::to_string(channel) + "@"
							+ std::to_string(chip) + "@"
							+ IO_Flow_Priority_Class::to_string(priority));
					}
					SLCGCReadTRQueue[channel][chip].Set_id(
						"SLC_GC_Read_TR_Queue@" + std::to_string(channel) + "@" + std::to_string(chip));
					SLCGCWriteTRQueue[channel][chip].Set_id(
						"SLC_GC_Write_TR_Queue@" + std::to_string(channel) + "@" + std::to_string(chip));
					SLCGCEraseTRQueue[channel][chip].Set_id(
						"SLC_GC_Erase_TR_Queue@" + std::to_string(channel) + "@" + std::to_string(chip));
				}
			}
		}

	TSU_Global_SLC_TLC::~TSU_Global_SLC_TLC()
	{
		for (unsigned int channel = 0; channel < channel_count; channel++) {
			for (unsigned int chip = 0; chip < chip_no_per_channel; chip++)
				delete[] TLCUserWriteTRQueue[channel][chip];
			delete[] TLCUserWriteTRQueue[channel];
			delete[] SLCGCReadTRQueue[channel];
			delete[] SLCGCWriteTRQueue[channel];
			delete[] SLCGCEraseTRQueue[channel];
		}
		delete[] TLCUserWriteTRQueue;
		delete[] SLCGCReadTRQueue;
		delete[] SLCGCWriteTRQueue;
		delete[] SLCGCEraseTRQueue;
	}

	void TSU_Global_SLC_TLC::Schedule()
	{
		opened_scheduling_reqs--;
		if (opened_scheduling_reqs > 0)
			return;
		if (opened_scheduling_reqs < 0)
			PRINT_ERROR("TSU_Global_SLC_TLC: Illegal scheduling status!")
		if (transaction_receive_slots.empty())
			return;

		for (auto transaction : transaction_receive_slots) {
			const flash_channel_ID_type channel = transaction->Address.ChannelID;
			const flash_chip_ID_type chip = transaction->Address.ChipID;
			switch (transaction->Type) {
				case Transaction_Type::READ:
					switch (transaction->Source) {
						case Transaction_Source_Type::CACHE:
						case Transaction_Source_Type::USERIO: {
							unsigned int priority = transaction->Priority_class == IO_Flow_Priority_Class::UNDEFINED
								? IO_Flow_Priority_Class::HIGH : static_cast<int>(transaction->Priority_class);
							UserReadTRQueue[channel][chip][priority].push_back(transaction);
							break;
						}
						case Transaction_Source_Type::MAPPING:
							MappingReadTRQueue[channel][chip].push_back(transaction);
							break;
						case Transaction_Source_Type::GC_WL:
							GCReadTRQueue[channel][chip].push_back(transaction);
							break;
						case Transaction_Source_Type::SLC_GC:
							SLCGCReadTRQueue[channel][chip].push_back(transaction);
							break;
						default:
							PRINT_ERROR("TSU_Global_SLC_TLC: unknown read transaction source!")
					}
					break;
				case Transaction_Type::WRITE:
					switch (transaction->Source) {
						case Transaction_Source_Type::CACHE:
						case Transaction_Source_Type::USERIO: {
							unsigned int priority = transaction->Priority_class == IO_Flow_Priority_Class::UNDEFINED
								? IO_Flow_Priority_Class::HIGH : static_cast<int>(transaction->Priority_class);
							Flash_Transaction_Queue*** queues = transaction->Tier == Flash_Data_Tier::TLC
								? TLCUserWriteTRQueue : UserWriteTRQueue;
							queues[channel][chip][priority].push_back(transaction);
							break;
						}
						case Transaction_Source_Type::MAPPING:
							MappingWriteTRQueue[channel][chip].push_back(transaction);
							break;
						case Transaction_Source_Type::GC_WL:
							GCWriteTRQueue[channel][chip].push_back(transaction);
							break;
						case Transaction_Source_Type::SLC_GC:
							SLCGCWriteTRQueue[channel][chip].push_back(transaction);
							break;
						default:
							PRINT_ERROR("TSU_Global_SLC_TLC: unknown write transaction source!")
					}
					break;
				case Transaction_Type::ERASE:
					if (transaction->Source == Transaction_Source_Type::SLC_GC)
						SLCGCEraseTRQueue[channel][chip].push_back(transaction);
					else
						GCEraseTRQueue[channel][chip].push_back(transaction);
					break;
				default:
					break;
			}
		}

		for (flash_channel_ID_type channel = 0; channel < channel_count; channel++) {
			if (_NVMController->Get_channel_status(channel) != BusChannelStatus::IDLE)
				continue;
			for (unsigned int attempt = 0; attempt < chip_no_per_channel; attempt++) {
				NVM::FlashMemory::Flash_Chip* chip =
					_NVMController->Get_chip(channel, Round_robin_turn_of_channel[channel]);
				if (_NVMController->GetChipStatus(chip) == ChipStatus::IDLE) {
					if (!service_read_transaction(chip)
						&& !service_write_transaction(chip)) {
						service_erase_transaction(chip);
					}
				}
				Round_robin_turn_of_channel[channel] =
					(flash_chip_ID_type)(Round_robin_turn_of_channel[channel] + 1) % chip_no_per_channel;
				if (_NVMController->Get_channel_status(channel) != BusChannelStatus::IDLE)
					break;
			}
		}
		transaction_receive_slots.clear();
	}

	void TSU_Global_SLC_TLC::Notify_tlc_gc_completion(
		const NVM::FlashMemory::Physical_Page_Address&)
	{
		pending_tlc_gc_completions++;
		Stats::TLC_gc_completions++;
	}

	bool TSU_Global_SLC_TLC::Can_start_background_slc_gc(
		const NVM::FlashMemory::Physical_Page_Address& source,
		const NVM::FlashMemory::Physical_Page_Address& target)
	{
		if (!transaction_receive_slots.empty())
			return false;
		if (_NVMController->Get_channel_status(source.ChannelID) != BusChannelStatus::IDLE
			|| _NVMController->Get_channel_status(target.ChannelID) != BusChannelStatus::IDLE)
			return false;
		if (_NVMController->GetChipStatus(
			_NVMController->Get_chip(source.ChannelID, source.ChipID)) != ChipStatus::IDLE
			|| _NVMController->GetChipStatus(
				_NVMController->Get_chip(target.ChannelID, target.ChipID)) != ChipStatus::IDLE)
			return false;
		return get_chan_count(source.ChannelID) == 0
			&& (target.ChannelID == source.ChannelID
				|| get_chan_count(target.ChannelID) == 0);
	}

	void TSU_Global_SLC_TLC::try_to_start_background_work()
	{
		ftl->GC_and_WL_Unit->Try_to_schedule_background_slc_gc();
	}

	void TSU_Global_SLC_TLC::record_post_tlc_gc_dispatch(Transaction_Source_Type source)
	{
		if (pending_tlc_gc_completions == 0)
			return;
		if (source == Transaction_Source_Type::USERIO || source == Transaction_Source_Type::CACHE)
			Stats::Post_TLC_GC_user_io += pending_tlc_gc_completions;
		else if (source == Transaction_Source_Type::GC_WL)
			Stats::Post_TLC_GC_tlc_gc += pending_tlc_gc_completions;
		else if (source == Transaction_Source_Type::SLC_GC)
			Stats::Post_TLC_GC_slc_gc += pending_tlc_gc_completions;
		else
			Stats::Post_TLC_GC_other += pending_tlc_gc_completions;
		pending_tlc_gc_completions = 0;
	}

	Flash_Transaction_Queue* TSU_Global_SLC_TLC::get_next_ready_user_write_queue(
		NVM::FlashMemory::Flash_Chip* chip)
	{
		const unsigned int order[] = {
			IO_Flow_Priority_Class::URGENT,
			IO_Flow_Priority_Class::HIGH,
			IO_Flow_Priority_Class::MEDIUM,
			IO_Flow_Priority_Class::LOW
		};
		for (unsigned int index = 0; index < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; index++) {
			Flash_Transaction_Queue* slc_queue =
				&UserWriteTRQueue[chip->ChannelID][chip->ChipID][order[index]];
			if (queue_has_ready_transaction(slc_queue))
				return slc_queue;
			Flash_Transaction_Queue* tlc_queue =
				&TLCUserWriteTRQueue[chip->ChannelID][chip->ChipID][order[index]];
			if (queue_has_ready_transaction(tlc_queue))
				return tlc_queue;
		}
		return NULL;
	}

	bool TSU_Global_SLC_TLC::queue_has_ready_transaction(
		Flash_Transaction_Queue* queue)
	{
		for (auto transaction : *queue) {
			if (transaction_is_ready(transaction))
				return true;
		}
		return false;
	}

	bool TSU_Global_SLC_TLC::service_read_transaction(
		NVM::FlashMemory::Flash_Chip* chip)
	{
		if (_NVMController->GetChipStatus(chip) != ChipStatus::IDLE)
			return false;

		const flash_channel_ID_type channel = chip->ChannelID;
		const flash_chip_ID_type chip_id = chip->ChipID;
		Flash_Transaction_Queue* source_queue = NULL;
		if (!MappingReadTRQueue[chip->ChannelID][chip->ChipID].empty()) {
			source_queue = &MappingReadTRQueue[chip->ChannelID][chip->ChipID];
		} else {
			source_queue = get_next_read_service_queue(chip);
			if (source_queue == NULL) {
				// Global priority, applied only to executable work on this chip:
				// mapping read, user read, mapping write, required TLC GC,
				// user write, urgent SLC GC, then background SLC GC.
				if (queue_has_ready_transaction(&MappingWriteTRQueue[channel][chip_id]))
					return false;
				if (!GCReadTRQueue[channel][chip_id].empty()) {
					source_queue = &GCReadTRQueue[channel][chip_id];
				} else if (queue_has_ready_transaction(&GCWriteTRQueue[channel][chip_id])
					|| queue_has_ready_transaction(&GCEraseTRQueue[channel][chip_id])) {
					return false;
				} else if (get_next_ready_user_write_queue(chip) != NULL) {
					return false;
				} else if (!SLCGCReadTRQueue[channel][chip_id].empty()) {
					source_queue = &SLCGCReadTRQueue[channel][chip_id];
				} else if (queue_has_ready_transaction(&SLCGCWriteTRQueue[channel][chip_id])
					|| queue_has_ready_transaction(&SLCGCEraseTRQueue[channel][chip_id])) {
					return false;
				} else {
					return false;
				}
			}
		}

		Transaction_Source_Type source = source_queue->front()->Source;
		bool issued = issue_command_to_chip(
			source_queue, NULL, Transaction_Type::READ, false);
		if (issued)
			record_post_tlc_gc_dispatch(source);
		return issued;
	}

	bool TSU_Global_SLC_TLC::service_write_transaction(
		NVM::FlashMemory::Flash_Chip* chip)
	{
		if (_NVMController->GetChipStatus(chip) != ChipStatus::IDLE)
			return false;

		const flash_channel_ID_type channel = chip->ChannelID;
		const flash_chip_ID_type chip_id = chip->ChipID;
		Flash_Transaction_Queue* source_queue = NULL;
		if (queue_has_ready_transaction(&MappingWriteTRQueue[channel][chip_id]))
			source_queue = &MappingWriteTRQueue[channel][chip_id];
		else if (queue_has_ready_transaction(&GCWriteTRQueue[channel][chip_id]))
			source_queue = &GCWriteTRQueue[channel][chip_id];
		else if (queue_has_ready_transaction(&GCEraseTRQueue[channel][chip_id]))
			return false;
		else {
			source_queue = get_next_ready_user_write_queue(chip);
			if (source_queue == NULL) {
				if (queue_has_ready_transaction(&SLCGCWriteTRQueue[channel][chip_id]))
					source_queue = &SLCGCWriteTRQueue[channel][chip_id];
				else if (queue_has_ready_transaction(&SLCGCEraseTRQueue[channel][chip_id]))
					return false;
				else
					return false;
			}
		}

		Transaction_Source_Type source = source_queue->front()->Source;
		bool issued = issue_command_to_chip(
			source_queue, NULL, Transaction_Type::WRITE, false);
		if (issued)
			record_post_tlc_gc_dispatch(source);
		return issued;
	}

	bool TSU_Global_SLC_TLC::service_erase_transaction(
		NVM::FlashMemory::Flash_Chip* chip)
	{
		if (_NVMController->GetChipStatus(chip) != ChipStatus::IDLE)
			return false;
		Flash_Transaction_Queue* source_queue = NULL;
		Flash_Transaction_Queue* tlc_erase =
			&GCEraseTRQueue[chip->ChannelID][chip->ChipID];
		Flash_Transaction_Queue* slc_erase =
			&SLCGCEraseTRQueue[chip->ChannelID][chip->ChipID];
		if (queue_has_ready_transaction(tlc_erase))
			source_queue = tlc_erase;
		else if (queue_has_ready_transaction(slc_erase))
			source_queue = slc_erase;
		else
			return false;

		Transaction_Source_Type source = source_queue->front()->Source;
		bool issued = issue_command_to_chip(
			source_queue, NULL, Transaction_Type::ERASE, false);
		if (issued)
			record_post_tlc_gc_dispatch(source);
		return issued;
	}

	uint64_t TSU_Global_SLC_TLC::get_chan_count(flash_channel_ID_type channel)
	{
		uint64_t count = TSU_Priority_OutOfOrder::get_chan_count(channel);
		for (unsigned int chip = 0; chip < chip_no_per_channel; chip++) {
			for (unsigned int priority = 0;
				priority < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; priority++)
				count += TLCUserWriteTRQueue[channel][chip][priority].size();
			count += SLCGCReadTRQueue[channel][chip].size();
			count += SLCGCWriteTRQueue[channel][chip].size();
			count += SLCGCEraseTRQueue[channel][chip].size();
		}
		return count;
	}

	uint64_t TSU_Global_SLC_TLC::get_chip_count(
		flash_channel_ID_type channel, flash_channel_ID_type chip)
	{
		uint64_t count = TSU_Priority_OutOfOrder::get_chip_count(channel, chip);
		for (unsigned int priority = 0;
			priority < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; priority++)
			count += TLCUserWriteTRQueue[channel][chip][priority].size();
		count += SLCGCReadTRQueue[channel][chip].size();
		count += SLCGCWriteTRQueue[channel][chip].size();
		count += SLCGCEraseTRQueue[channel][chip].size();
		return count;
	}

	void TSU_Global_SLC_TLC::Report_results_in_XML(
		std::string name_prefix, Utils::XmlWriter& xmlwriter)
	{
		name_prefix += ".TSU";
		xmlwriter.Write_open_tag(name_prefix);
		TSU_Base::Report_results_in_XML(name_prefix, xmlwriter);
		for (unsigned int channel = 0; channel < channel_count; channel++) {
			for (unsigned int chip = 0; chip < chip_no_per_channel; chip++) {
				for (unsigned int priority = 0;
					priority < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; priority++) {
					std::string priority_name = IO_Flow_Priority_Class::to_string(priority);
					UserReadTRQueue[channel][chip][priority].Report_results_in_XML(
						name_prefix + ".User_Read_TR_Queue.Priority." + priority_name, xmlwriter);
					UserWriteTRQueue[channel][chip][priority].Report_results_in_XML(
						name_prefix + ".SLC_User_Write_TR_Queue.Priority." + priority_name, xmlwriter);
					TLCUserWriteTRQueue[channel][chip][priority].Report_results_in_XML(
						name_prefix + ".TLC_User_Write_TR_Queue.Priority." + priority_name, xmlwriter);
				}
				MappingReadTRQueue[channel][chip].Report_results_in_XML(
					name_prefix + ".Mapping_Read_TR_Queue", xmlwriter);
				MappingWriteTRQueue[channel][chip].Report_results_in_XML(
					name_prefix + ".Mapping_Write_TR_Queue", xmlwriter);
				GCReadTRQueue[channel][chip].Report_results_in_XML(
					name_prefix + ".GC_Read_TR_Queue", xmlwriter);
				GCWriteTRQueue[channel][chip].Report_results_in_XML(
					name_prefix + ".GC_Write_TR_Queue", xmlwriter);
				GCEraseTRQueue[channel][chip].Report_results_in_XML(
					name_prefix + ".GC_Erase_TR_Queue", xmlwriter);
				SLCGCReadTRQueue[channel][chip].Report_results_in_XML(
					name_prefix + ".SLC_GC_Read_TR_Queue", xmlwriter);
				SLCGCWriteTRQueue[channel][chip].Report_results_in_XML(
					name_prefix + ".SLC_GC_Write_TR_Queue", xmlwriter);
				SLCGCEraseTRQueue[channel][chip].Report_results_in_XML(
					name_prefix + ".SLC_GC_Erase_TR_Queue", xmlwriter);
			}
		}
		xmlwriter.Write_close_tag();
	}
}
