#ifndef TSU_H
#define TSU_H

#include <list>
#include "../sim/Sim_Defs.h"
#include "../sim/Sim_Object.h"
#include "../nvm_chip/flash_memory/Flash_Chip.h"
#include "../sim/Sim_Reporter.h"
#include "FTL.h"
#include "NVM_PHY_ONFI_NVDDR2.h"
#include "Flash_Transaction_Queue.h"

namespace SSD_Components
{
enum class Flash_Scheduling_Type
{
	OUT_OF_ORDER,
	PRIORITY_OUT_OF_ORDER,
	GLOBAL_SLC_TLC,
	FLIN
};
class FTL;
class TSU_Base : public MQSimEngine::Sim_Object
{
public:
	TSU_Base(const sim_object_id_type &id, FTL *ftl, NVM_PHY_ONFI_NVDDR2 *NVMController, Flash_Scheduling_Type Type,
			 unsigned int Channel_no, unsigned int chip_no_per_channel, unsigned int DieNoPerChip, unsigned int PlaneNoPerDie,
			 bool EraseSuspensionEnabled, bool ProgramSuspensionEnabled,
			 sim_time_type WriteReasonableSuspensionTimeForRead,
			 sim_time_type EraseReasonableSuspensionTimeForRead,
			 sim_time_type EraseReasonableSuspensionTimeForWrite);
	virtual ~TSU_Base();
	void Setup_triggers();

	/*When an MQSim needs to send a set of transactions for execution, the following 
		* three funcitons should be invoked in this order:
		* Prepare_for_transaction_submit()
		* Submit_transaction(transaction)
		* .....
		* Submit_transaction(transaction)
		* Schedule()
		*
		* The above mentioned mechanism helps to exploit die-level and plane-level parallelism.
		* More precisely, first the transactions are queued and then, when the Schedule function
		* is invoked, the TSU has that opportunity to schedule them together to exploit multiplane
		* and die-interleaved execution.
		*/
	void Prepare_for_transaction_submit()
	{
		opened_scheduling_reqs++;
		if (opened_scheduling_reqs > 1)
		{
			return;
		}
		transaction_receive_slots.clear();
	}



	void Submit_transaction(NVM_Transaction_Flash *transaction)
	{	
		// if(Transaction_Source_Type::USERIO == transaction->Source || Transaction_Source_Type::CACHE == transaction->Source){
		// 	if(transaction->Type == Transaction_Type::WRITE){
				
		// 	}
		// }
		restWrite++;
		// if(transaction->Type == Transaction_Type::WRITE){
			
		// 	if(transaction->Stream_id == 3 && transaction->LPA == 207362){
		// 		readCount++;
		// 		std::cout << " tr " << transaction <<" " <<readCount<< std::endl;
		// 		std::cout << transaction->Stream_id << " " << transaction->LPA << std::endl;
		// 		std::cout <<"read " << (transaction->Type == Transaction_Type::READ) <<std::endl;
		// 		std::cout <<"write " << (transaction->Type == Transaction_Type::WRITE) <<std::endl;
		// 		std::cout << reinterpret_cast<NVM_Transaction_Flash_WR*>(transaction)->RelatedRead << std::endl;
		// 		// throw "transaction->Type == Transaction_Type::READ";
		// 		if(readCount == 24){
		// 			reinterpret_cast<NVM_Transaction_Flash_WR*>(transaction)->RelatedRead->track = true;
		// 			checkFlag = true;
		// 		}
		// 	}
		// }else{
		// 	static int interCount = 0;
		// 	if(transaction->Stream_id == 3 && transaction->LPA == 207362){
		// 		interCount++;
		// 		std::cout << transaction->Address.ChannelID << " " << transaction->Address.ChipID << std::endl;
		// 		std::cout << transaction << " read  insert " << interCount << std::endl;
		// 	}
		// }
		if(transaction->track){
			std::cout << "insert to tsu" << std::endl;
		}
		if(transaction->Type == Transaction_Type::READ && reinterpret_cast<NVM_Transaction_Flash_RD*>(transaction)->RelatedWrite != NULL){
			transaction->Priority_class = IO_Flow_Priority_Class::Priority::LOW;
		}
		
		transaction_receive_slots.push_back(transaction);
	}

	virtual bool find_mapping_io(flash_channel_ID_type ChannelID, flash_chip_ID_type ChipID, stream_id_type stream, LPA_type lpa){
		return false;
	}

	/* Shedules the transactions currently stored in inputTransactionSlots. The transactions could
		* be mixes of reads, writes, and erases.
		*/
	virtual void Schedule() = 0;
	virtual void Report_results_in_XML(std::string name_prefix, Utils::XmlWriter &xmlwriter);
	virtual void Notify_tlc_gc_completion(const NVM::FlashMemory::Physical_Page_Address&) {}
	virtual bool Can_start_background_slc_gc(
		const NVM::FlashMemory::Physical_Page_Address&,
		const NVM::FlashMemory::Physical_Page_Address&) { return false; }

	virtual uint64_t get_chan_count(flash_channel_ID_type ChannelID){
		return 0;
	}

	virtual uint64_t get_chip_count(flash_channel_ID_type ChannelID, flash_channel_ID_type){
		return 0;
	}
	
protected:
	unsigned long long restWrite = 0;
	FTL *ftl;
	NVM_PHY_ONFI_NVDDR2 *_NVMController;
	Flash_Scheduling_Type type;
	unsigned int channel_count;
	unsigned int chip_no_per_channel;
	unsigned int die_no_per_chip;
	unsigned int plane_no_per_die;
	bool eraseSuspensionEnabled, programSuspensionEnabled;
	sim_time_type writeReasonableSuspensionTimeForRead;
	sim_time_type eraseReasonableSuspensionTimeForRead; //the time period
	sim_time_type eraseReasonableSuspensionTimeForWrite;
	flash_chip_ID_type *Round_robin_turn_of_channel; //Used for round-robin service of the chips in channels

	static TSU_Base *_my_instance;
	std::list<NVM_Transaction_Flash *> transaction_receive_slots;  //Stores the transactions that are received for sheduling
	std::list<NVM_Transaction_Flash *> transaction_dispatch_slots; //Used to submit transactions to the channel controller
	virtual bool service_read_transaction(NVM::FlashMemory::Flash_Chip *chip) = 0;
	virtual bool service_write_transaction(NVM::FlashMemory::Flash_Chip *chip) = 0;
	virtual bool service_erase_transaction(NVM::FlashMemory::Flash_Chip *chip) = 0;
	virtual void try_to_start_background_work() {}
	bool issue_command_to_chip(Flash_Transaction_Queue *sourceQueue1, Flash_Transaction_Queue *sourceQueue2, Transaction_Type transactionType, bool suspensionRequired);
	static void handle_transaction_serviced_signal_from_PHY(NVM_Transaction_Flash *transaction);
	static void handle_channel_idle_signal(flash_channel_ID_type);
	static void handle_chip_idle_signal(NVM::FlashMemory::Flash_Chip *chip);
	int opened_scheduling_reqs;
	bool checkFlag = false;
		void process_chip_requests(NVM::FlashMemory::Flash_Chip* chip)
		{
			if(checkFlag && chip->ChannelID == 0 && chip->ChipID == 2){
				std::cout << "do chip "<< std::endl;
			}
			if (!_my_instance->service_read_transaction(chip)) {
				if (!_my_instance->service_write_transaction(chip)) {
					_my_instance->service_erase_transaction(chip);
				}
			}

			for (flash_channel_ID_type channelID = 0; channelID < channel_count; channelID++) {
				if (_NVMController->Get_channel_status(channelID) != BusChannelStatus::IDLE) {
					continue;
				}
				for (unsigned int i = 0; i < chip_no_per_channel; i++) {
					NVM::FlashMemory::Flash_Chip *chip =
						_NVMController->Get_chip(channelID, Round_robin_turn_of_channel[channelID]);
					// Advance after every attempt so a busy or empty chip cannot
					// hide ready work on another chip in the same channel.
					if (_NVMController->GetChipStatus(chip) == ChipStatus::IDLE) {
						if (!service_read_transaction(chip)
							&& !service_write_transaction(chip)) {
							service_erase_transaction(chip);
						}
					}
					Round_robin_turn_of_channel[channelID] =
						(flash_chip_ID_type)(Round_robin_turn_of_channel[channelID] + 1) % chip_no_per_channel;
					if (_NVMController->Get_channel_status(channelID) != BusChannelStatus::IDLE) {
						break;
					}
				}
			}
			_my_instance->try_to_start_background_work();
		}

	unsigned long long readCount = 0;
protected:
	bool transaction_is_ready(NVM_Transaction_Flash* transaction);

	unsigned long long dietodie = 0;
};
} // namespace SSD_Components

#endif //TSU_H
