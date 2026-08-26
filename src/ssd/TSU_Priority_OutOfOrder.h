#ifndef TSU_PRIORITY_OUTOFORDER_H_
#define TSU_PRIORITY_OUTOFORDER_H_

#include <list>
#include "TSU_Base.h"
#include "NVM_Transaction_Flash.h"
#include "NVM_PHY_ONFI_NVDDR2.h"
#include "FTL.h"

namespace SSD_Components
{
class FTL;

/*
	* This class implements a transaction scheduling unit which supports:
	* 1. Out-of-order execution of flash transactions, similar to the Sprinkler proposal
	*    described in "Jung et al., Sprinkler: Maximizing resource utilization in many-chip
	*    solid state disks, HPCA, 2014".
	* 2. Program and erase suspension, similar to the proposal described in "G. Wu and X. He,
	*    Reducing SSD read latency via NAND flash program and erase suspension, FAST 2012".
	*/
class TSU_Priority_OutOfOrder : public TSU_Base
{
public:
    TSU_Priority_OutOfOrder(const sim_object_id_type &id,
                            FTL *ftl,
                            NVM_PHY_ONFI_NVDDR2 *NVMController,
                            unsigned int Channel_no,
                            unsigned int chip_no_per_channel,
                            unsigned int DieNoPerChip,
                            unsigned int PlaneNoPerDie,
                            sim_time_type WriteReasonableSuspensionTimeForRead,
                            sim_time_type EraseReasonableSuspensionTimeForRead,
                            sim_time_type EraseReasonableSuspensionTimeForWrite,
                            bool EraseSuspensionEnabled,
                            bool ProgramSuspensionEnabled,
                            Flash_Scheduling_Type scheduling_type = Flash_Scheduling_Type::PRIORITY_OUT_OF_ORDER);
    ~TSU_Priority_OutOfOrder();
    
	    void Schedule() override;

    void Start_simulation();
    void Validate_simulation_config();
    void Execute_simulator_event(MQSimEngine::Sim_Event *);
	    void Report_results_in_XML(std::string name_prefix, Utils::XmlWriter &xmlwriter) override;

    virtual bool find_mapping_io(flash_channel_ID_type ChannelID, flash_chip_ID_type ChipID, stream_id_type stream, LPA_type lpa);

    virtual uint64_t get_chan_count(flash_channel_ID_type ChannelID);

    virtual uint64_t get_chip_count(flash_channel_ID_type ChannelID, flash_channel_ID_type);
protected:
	    Flash_Transaction_Queue ***UserReadTRQueue;
	    Flash_Transaction_Queue ***UserWriteTRQueue;
	    Flash_Transaction_Queue **GCReadTRQueue;
	    Flash_Transaction_Queue **GCWriteTRQueue;
	    Flash_Transaction_Queue **GCEraseTRQueue;
	    Flash_Transaction_Queue **MappingReadTRQueue;
	    Flash_Transaction_Queue **MappingWriteTRQueue;
	    IO_Flow_Priority_Class::Priority **nextPriorityClassRead;
	    IO_Flow_Priority_Class::Priority **nextPriorityClassWrite;
	    int **currentWeightRead;
	    int **currentWeightWrite;
	    bool service_read_transaction(NVM::FlashMemory::Flash_Chip *chip) override;
	    bool service_write_transaction(NVM::FlashMemory::Flash_Chip *chip) override;
	    bool service_erase_transaction(NVM::FlashMemory::Flash_Chip *chip) override;
	    Flash_Transaction_Queue *get_next_read_service_queue(NVM::FlashMemory::Flash_Chip *chip);
	    Flash_Transaction_Queue *get_next_write_service_queue(NVM::FlashMemory::Flash_Chip *chip);
    
    
};
} // namespace SSD_Components

#endif // TSU_PRIORITY_OUTOFORDER_H_
