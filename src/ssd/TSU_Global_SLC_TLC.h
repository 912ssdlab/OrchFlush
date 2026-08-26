#ifndef TSU_GLOBAL_SLC_TLC_H
#define TSU_GLOBAL_SLC_TLC_H

#include "TSU_Priority_OutOfOrder.h"

namespace SSD_Components
{
	class TSU_Global_SLC_TLC : public TSU_Priority_OutOfOrder
	{
	public:
		TSU_Global_SLC_TLC(const sim_object_id_type& id, FTL* ftl,
			NVM_PHY_ONFI_NVDDR2* controller, unsigned int channel_count,
			unsigned int chip_no_per_channel, unsigned int die_no_per_chip,
			unsigned int plane_no_per_die,
			sim_time_type write_suspend_for_read,
			sim_time_type erase_suspend_for_read,
			sim_time_type erase_suspend_for_write,
			bool erase_suspension_enabled, bool program_suspension_enabled);
		~TSU_Global_SLC_TLC() override;

		void Schedule() override;
		void Report_results_in_XML(std::string name_prefix, Utils::XmlWriter& xmlwriter) override;
		void Notify_tlc_gc_completion(const NVM::FlashMemory::Physical_Page_Address&) override;
		bool Can_start_background_slc_gc(
			const NVM::FlashMemory::Physical_Page_Address& source,
			const NVM::FlashMemory::Physical_Page_Address& target) override;
		uint64_t get_chan_count(flash_channel_ID_type channel) override;
		uint64_t get_chip_count(flash_channel_ID_type channel, flash_channel_ID_type chip) override;

	protected:
		bool service_read_transaction(NVM::FlashMemory::Flash_Chip* chip) override;
		bool service_write_transaction(NVM::FlashMemory::Flash_Chip* chip) override;
		bool service_erase_transaction(NVM::FlashMemory::Flash_Chip* chip) override;
		void try_to_start_background_work() override;

	private:
		Flash_Transaction_Queue*** TLCUserWriteTRQueue;
		Flash_Transaction_Queue** SLCGCReadTRQueue;
		Flash_Transaction_Queue** SLCGCWriteTRQueue;
		Flash_Transaction_Queue** SLCGCEraseTRQueue;
		unsigned long pending_tlc_gc_completions = 0;
		Flash_Transaction_Queue* get_next_ready_user_write_queue(
			NVM::FlashMemory::Flash_Chip* chip);
		bool queue_has_ready_transaction(Flash_Transaction_Queue* queue);
		void record_post_tlc_gc_dispatch(Transaction_Source_Type source);
	};
}

#endif
