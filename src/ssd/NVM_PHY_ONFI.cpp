#include "NVM_PHY_ONFI.h"
#include "Stats.h"

namespace SSD_Components {
	void NVM_PHY_ONFI::ConnectToTransactionServicedSignal(TransactionServicedHandlerType function)
	{
		connectedTransactionServicedHandlers.push_back(function);
	}

	/*
	* Different FTL components maybe waiting for a transaction to be finished:
	* HostInterface: For user reads and writes
	* Address_Mapping_Unit: For mapping reads and writes
	* TSU: For the reads that must be finished for partial writes (first read non updated parts of page data and then merge and write them into the new page)
	* GarbageCollector: For gc reads, writes, and erases
	*/
	void NVM_PHY_ONFI::broadcastTransactionServicedSignal(NVM_Transaction_Flash* transaction)
	{
		const bool is_slc = transaction->Address.BlockID >= tlc_block_no_per_plane;
		switch (transaction->Type) {
			case Transaction_Type::READ:
				if (is_slc)
					Stats::SLC_physical_reads++;
				else
					Stats::TLC_physical_reads++;
				break;
			case Transaction_Type::WRITE:
				if (is_slc)
					Stats::SLC_physical_writes++;
				else
					Stats::TLC_physical_writes++;
				break;
			case Transaction_Type::ERASE:
				if (is_slc)
					Stats::SLC_physical_erases++;
				else
					Stats::TLC_physical_erases++;
				break;
			default:
				break;
		}

		std::vector<std::pair<flash_channel_ID_type, flash_chip_ID_type>> mayDo;
		for (std::vector<TransactionServicedHandlerType>::iterator it = connectedTransactionServicedHandlers.begin();
			it != connectedTransactionServicedHandlers.end(); it++) {
			if(transaction->Type == Transaction_Type::READ){
				NVM_Transaction_Flash_RD *readtr = (NVM_Transaction_Flash_RD *)transaction;
				if(readtr->RelatedWrite && readtr->Address.ChannelID != readtr->RelatedWrite->Address.ChannelID){
					mayDo.push_back({readtr->RelatedWrite->Address.ChannelID, readtr->RelatedWrite->Address.ChipID});
				}
				if(readtr->RelatedErase && readtr->Address.ChannelID != reinterpret_cast<NVM_Transaction_Flash_ER*>(readtr->RelatedErase)->Address.ChannelID){
					mayDo.push_back({reinterpret_cast<NVM_Transaction_Flash_ER*>(readtr->RelatedErase)->Address.ChannelID, reinterpret_cast<NVM_Transaction_Flash_ER*>(readtr->RelatedErase)->Address.ChipID});
				}
			}
			(*it)(transaction);
		}
		
		
		delete transaction;//This transaction has been consumed and no more needed

		for(auto &key : mayDo){
			if(Get_channel_status(key.first) == BusChannelStatus::IDLE){
				NVM::FlashMemory::Flash_Chip* chip = Get_chip(key.first, key.second);
				if(GetChipStatus(chip) == ChipStatus::IDLE){
					broadcastChipIdleSignal(chip);
				}
			}
		}
	}

	void NVM_PHY_ONFI::ConnectToChannelIdleSignal(ChannelIdleHandlerType function)
	{
		connectedChannelIdleHandlers.push_back(function);
	}

	void NVM_PHY_ONFI::broadcastChannelIdleSignal(flash_channel_ID_type channelID)
	{
		for (std::vector<ChannelIdleHandlerType>::iterator it = connectedChannelIdleHandlers.begin();
			it != connectedChannelIdleHandlers.end(); it++) {
			(*it)(channelID);
		}
	}

	void NVM_PHY_ONFI::ConnectToChipIdleSignal(ChipIdleHandlerType function)
	{
		connectedChipIdleHandlers.push_back(function);
	}

	void NVM_PHY_ONFI::broadcastChipIdleSignal(NVM::FlashMemory::Flash_Chip* chip)
	{
		for (std::vector<ChipIdleHandlerType>::iterator it = connectedChipIdleHandlers.begin();
			it != connectedChipIdleHandlers.end(); it++) {
			(*it)(chip);
		}
	}
}
