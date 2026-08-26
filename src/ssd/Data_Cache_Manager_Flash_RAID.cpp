#include "Data_Cache_Manager_Flash_RAID.h"
#include "FTL.h"
#include "SSD_Defs.h"
#include "Data_Cache_Manager_Base.h"
#include <stdexcept>
#include "../nvm_chip/NVM_Types.h"
#include "NVM_Transaction_Flash_RD.h"
#include "NVM_Transaction_Flash_WR.h"
#include "FTL.h"
#include "Address_Mapping_Unit_Page_Level_And_RAID.h"

namespace SSD_Components{

Data_Cache_Manager_Flash_RAID::Data_Cache_Manager_Flash_RAID(const sim_object_id_type& id, Host_Interface_Base* host_interface, NVM_Firmware* firmware, NVM_PHY_ONFI* flash_controller,
    unsigned int total_capacity_in_bytes,
    unsigned int dram_row_size, unsigned int dram_data_rate, unsigned int dram_busrt_size, sim_time_type dram_tRCD, sim_time_type dram_tCL, sim_time_type dram_tRP,
    Caching_Mode* caching_mode_per_input_stream, Cache_Sharing_Mode sharing_mode,unsigned int stream_count,
    unsigned int sector_no_per_page_, unsigned int back_pressure_buffer_max_depth)
    : Data_Cache_Manager_Base(id, host_interface, firmware, dram_row_size, dram_data_rate, dram_busrt_size, dram_tRCD, dram_tCL, dram_tRP, caching_mode_per_input_stream, sharing_mode, stream_count),
    flash_controller(flash_controller), capacity_in_bytes(total_capacity_in_bytes), sector_no_per_page(sector_no_per_page_),	memory_channel_is_busy(false),
    dram_execution_list_turn(0)
{
    capacity_in_pages = capacity_in_bytes / (SECTOR_SIZE_IN_BYTE * sector_no_per_page);
    ppc_cache = new Data_Cache_Flash(capacity_in_pages);
}

Data_Cache_Manager_Flash_RAID::~Data_Cache_Manager_Flash_RAID(){
    std::cout <<"gcCacheC " <<  gcCacheC << std::endl;
}

void Data_Cache_Manager_Flash_RAID::service_dram_access_request(Memory_Transfer_Info* request_info){
    if (memory_channel_is_busy) {
        dram_execution_queue.push(request_info);
    } else {
        Simulator->Register_sim_event(Simulator->Time() + estimate_dram_access_time(request_info->Size_in_bytes, dram_row_size,
            dram_busrt_size, dram_burst_transfer_time_ddr, dram_tRCD, dram_tCL, dram_tRP),
            this, request_info, static_cast<int>(request_info->next_event_type));
        memory_channel_is_busy = true;
        //dram_execution_list_turn = request_info->Stream_id;
    }
}

void Data_Cache_Manager_Flash_RAID::Execute_simulator_event(MQSimEngine::Sim_Event* ev){

    Data_Cache_Simulation_Event_Type eventType = (Data_Cache_Simulation_Event_Type)ev->Type;
    Memory_Transfer_Info* transfer_info = (Memory_Transfer_Info*)ev->Parameters;

    switch (eventType)
    {
        case Data_Cache_Simulation_Event_Type::MEMORY_READ_FOR_USERIO_FINISHED://A user read is service from DRAM cache
        case Data_Cache_Simulation_Event_Type::MEMORY_WRITE_FOR_USERIO_FINISHED:
            ((User_Request*)(transfer_info)->Related_request)->Sectors_serviced_from_cache -= transfer_info->Size_in_bytes / SECTOR_SIZE_IN_BYTE;
            if ((User_Request*)(transfer_info)->Related_request && is_user_request_finished((User_Request*)(transfer_info)->Related_request))
                broadcast_user_request_serviced_signal(((User_Request*)(transfer_info)->Related_request));
            break;
        case Data_Cache_Simulation_Event_Type::MEMORY_READ_FOR_CACHE_EVICTION_FINISHED://Reading data from DRAM and writing it back to the flash storage
            static_cast<FTL*>(nvm_firmware)->Address_Mapping_Unit->Translate_lpa_to_ppa_and_dispatch(*((std::list<NVM_Transaction*>*)(transfer_info->Related_request)));
            delete (std::list<NVM_Transaction*>*)transfer_info->Related_request;
            break;
        case Data_Cache_Simulation_Event_Type::MEMORY_WRITE_FOR_CACHE_FINISHED://The recently read data from flash is written back to memory to support future user read requests
            break;
    }
    delete transfer_info;

    memory_channel_is_busy = false;
    
    if (dram_execution_queue.size() > 0)	{
        Memory_Transfer_Info* transfer_info = dram_execution_queue.front();
        dram_execution_queue.pop();
        Simulator->Register_sim_event(Simulator->Time() + estimate_dram_access_time(transfer_info->Size_in_bytes, dram_row_size, dram_busrt_size,
            dram_burst_transfer_time_ddr, dram_tRCD, dram_tCL, dram_tRP),
            this, transfer_info, static_cast<int>(transfer_info->next_event_type));
        memory_channel_is_busy = true;
    }
}

bool Data_Cache_Manager_Flash_RAID::is_exist(stream_id_type stream, LPA_type lpa){
    return ppc_cache->Exists(stream, lpa);
}

void Data_Cache_Manager_Flash_RAID::gc_eviction(std::vector<std::pair<stream_id_type, LPA_type>> keys, NVM_Transaction_Flash_ER* gc_wl_erase_tr){
    // return;
    std::list<NVM_Transaction*>* evicted_cache_slots = new std::list<NVM_Transaction*>;
    unsigned int cache_eviction_read_size_in_sectors = 0;//The size of data evicted from cache
    Address_Mapping_Unit_Page_Level_And_RAID *addrMap = dynamic_cast<Address_Mapping_Unit_Page_Level_And_RAID*>(static_cast<FTL*>(nvm_firmware)->Address_Mapping_Unit);
    for(auto &key : keys){
        stream_id_type stream = key.first;
        LPA_type lpa = key.second;
        
        if(ppc_cache->Exists(stream, lpa)){
            ++gcCacheC;
            LPA_type key = LPN_TO_UNIQUE_KEY(stream, lpa);
            Data_Cache_Slot_Type evicted_slot = ppc_cache->Evict_one_slot(key);
            if (evicted_slot.Status == Cache_Slot_Status::DIRTY_NO_FLASH_WRITEBACK) {
                std::vector<NVM_Transaction_Flash*> trans = create_evict_trans(evicted_slot, nullptr, gc_wl_erase_tr);
                
                NVM_Transaction_Flash_WR* writeTr = reinterpret_cast<NVM_Transaction_Flash_WR*>(trans[0]);
                
                addrMap->Semilate_page(writeTr->Stream_id, writeTr->LPA);

                evicted_cache_slots->push_back(writeTr);
                cache_eviction_read_size_in_sectors += count_sector_no_from_status_bitmap(evicted_slot.State_bitmap_of_existing_sectors);
            }
        }else{
            throw "gc_eviction";
        }
    }
    if(evicted_cache_slots->size() > 0){
        // std::cout << evicted_cache_slots->size() <<" "<<(evicted_cache_slots->back()->Source == Transaction_Source_Type::GC_WL)  << std::endl;
        Memory_Transfer_Info* read_transfer_info = new Memory_Transfer_Info;
        read_transfer_info->Size_in_bytes = cache_eviction_read_size_in_sectors * SECTOR_SIZE_IN_BYTE;
        read_transfer_info->Related_request = evicted_cache_slots;
        read_transfer_info->next_event_type = Data_Cache_Simulation_Event_Type::MEMORY_READ_FOR_CACHE_EVICTION_FINISHED;
        read_transfer_info->Stream_id = 0;
        service_dram_access_request(read_transfer_info);
    }else{
        delete evicted_cache_slots;
    }
    
}

std::vector<NVM_Transaction_Flash*> Data_Cache_Manager_Flash_RAID::create_evict_trans(const Data_Cache_Slot_Type evicted_slot, User_Request* user_request, NVM_Transaction_Flash_ER* gc_wl_erase_tr){
    std::vector<NVM_Transaction_Flash*> ret;
    Address_Mapping_Unit_Page_Level_And_RAID *addrMap = dynamic_cast<Address_Mapping_Unit_Page_Level_And_RAID*>(static_cast<FTL*>(nvm_firmware)->Address_Mapping_Unit);
    NVM_Transaction_Flash_WR *write = new NVM_Transaction_Flash_WR(Transaction_Source_Type::CACHE, evicted_slot.streamid, count_sector_no_from_status_bitmap(evicted_slot.State_bitmap_of_existing_sectors) * SECTOR_SIZE_IN_BYTE, evicted_slot.LPA, user_request, IO_Flow_Priority_Class::HIGH, evicted_slot.Content, evicted_slot.State_bitmap_of_existing_sectors, evicted_slot.Timestamp);
    ret.emplace_back(write);
    write->RelatedRead = (NVM_Transaction_Flash_RD*)1;
    int *relatedCount = new int;
    *relatedCount = 0;

    int semiCount = addrMap->get_Semilate_page_count(evicted_slot.streamid, evicted_slot.LPA);
    int NotsemiCount = addrMap->get_Not_Semilate_page_count(evicted_slot.streamid, evicted_slot.LPA);
    if(semiCount == 0){
        if(user_request == nullptr)
            std::cout << semiCount<< " "<< NotsemiCount<<" " <<evicted_slot.streamid << " " << evicted_slot.LPA << " "<<10 << std::endl;
        write->RelatedRead = NULL;
        return  ret;
    }
    Transaction_Source_Type source;
    source = Transaction_Source_Type::CACHE;
    
    write->hasRelatedRead = true;
    write->relatedCount = relatedCount;
    write->cacheEj = true;

    static_cast<FTL*>(nvm_firmware)->TSU->Prepare_for_transaction_submit();
    if(semiCount >= NotsemiCount){
        std::vector<LPA_type> lpas = addrMap->get_Not_Semilate_pages(evicted_slot.streamid, evicted_slot.LPA);
        NVM_Transaction_Flash_RD *read = NULL;
        for(auto lpa : lpas){
            if(lpa != NO_LPA){
                stream_id_type stream = (lpa & (NO_LPA << 56)) >> 56;
                lpa = UNIQUE_KEY_TO_LPN(stream, lpa);
                page_status_type pageStatus = addrMap->Get_page_status(true ,stream, lpa);
                read = new NVM_Transaction_Flash_RD(source, stream, 
                count_sector_no_from_status_bitmap(pageStatus) * SECTOR_SIZE_IN_BYTE, lpa, NO_PPA,
                user_request, IO_Flow_Priority_Class::HIGH, evicted_slot.Content, 
                pageStatus, evicted_slot.Timestamp);

                
                read->RelatedWrite = write;
                read->relatedCount = relatedCount;
                (*relatedCount) += 1;
                read->cacheEj = true;
                ret.emplace_back(read);
            }
        }
    }else{
        NVM_Transaction_Flash_RD *read = NULL;
        page_status_type pageStatus = addrMap->Get_page_status(true ,evicted_slot.streamid, evicted_slot.LPA);
        if(pageStatus != UNWRITTEN_LOGICAL_PAGE){
            read = new NVM_Transaction_Flash_RD(source, evicted_slot.streamid, 
            count_sector_no_from_status_bitmap(pageStatus) * SECTOR_SIZE_IN_BYTE, evicted_slot.LPA, NO_PPA,
            user_request, IO_Flow_Priority_Class::HIGH, evicted_slot.Content, 
            pageStatus, evicted_slot.Timestamp);
            read->cacheEj = true;
            read->RelatedWrite = write;
            read->relatedCount = relatedCount;
            (*relatedCount) += 1;
            ret.emplace_back(read);
        }
        NVM::FlashMemory::Physical_Page_Address address;
        std::vector<PPA_type> ppas = addrMap->get_Semilate_pages(evicted_slot.streamid, evicted_slot.LPA);
        for(auto ppa : ppas){
            addrMap->Convert_ppa_to_address(ppa, address);
            read = new NVM_Transaction_Flash_RD(source, NO_STREAM, sector_no_per_page * SECTOR_SIZE_IN_BYTE,
								NO_LPA, ppa, address, user_request, 0, NULL, 0, INVALID_TIME_STAMP);
            
            read->Priority_class = IO_Flow_Priority_Class::HIGH;
            read->Physical_address_determined = true;
            read->relatedCount = relatedCount;
            (*relatedCount) += 1;
            read->RelatedWrite = write;
            read->cacheEj = true;
            if(user_request == nullptr){
                if(gc_wl_erase_tr->Address.ChannelID != address.ChannelID)
                    std::cout << semiCount << " " << gc_wl_erase_tr->Address.ChannelID << " " << address.ChannelID << std::endl;
                read->RelatedErase = gc_wl_erase_tr;
                gc_wl_erase_tr->Page_movement_activities.push_back(read);
                read->Source = Transaction_Source_Type::GC_WL;
                read->cacheEj = true;
            }else{
                static_cast<FTL*>(nvm_firmware)->BlockManager->Read_transaction_issued(address);
            }
            
            ret.emplace_back(read);
        }
    }
    static_cast<FTL*>(nvm_firmware)->TSU->Schedule();
    if(user_request == NULL){
        std::list<NVM_Transaction*> submits;
        for(int i = 1; i < ret.size(); i++){
            submits.emplace_back(ret[i]);
        }
        static_cast<FTL*>(nvm_firmware)->Address_Mapping_Unit->Translate_lpa_to_ppa_and_dispatch(submits);
    }
    
    return ret;
}

void Data_Cache_Manager_Flash_RAID::process_new_user_request(User_Request* user_request){
    unsigned int dram_write_size_in_sectors = 0;//The size of data written to DRAM (must be >= flash_written_back_write_size_in_sectors)
    unsigned int cache_eviction_read_size_in_sectors = 0;//The size of data evicted from cache
    
    if (user_request->Transaction_list.size() == 0) {
        return;
    }
    
    std::list<NVM_Transaction*>* evicted_cache_slots = new std::list<NVM_Transaction*>;
    std::list<NVM_Transaction*> readTrans;

    if (user_request->Type == UserRequestType::WRITE){
        /*update the ppc in cache and may evtcion*/
        std::list<NVM_Transaction*>::iterator it = user_request->Transaction_list.begin();
        while (it != user_request->Transaction_list.end()){
            
            NVM_Transaction_Flash_WR* tr = (NVM_Transaction_Flash_WR*)(*it);
            LPA_type raidLPA = dynamic_cast<Address_Mapping_Unit_Page_Level_And_RAID*>(static_cast<FTL*>(nvm_firmware)->Address_Mapping_Unit)->Get_And_Create_RAIDID(tr->Stream_id, tr->LPA);
            if(raidLPA == NO_LPA){
                throw std::string("raidLPA == NO_LPA in Data_Cache_Manager_Flash_RAID::process_new_user_request");
            }
            stream_id_type stream = (raidLPA & (NO_LPA << 56)) >> 56;
            raidLPA = UNIQUE_KEY_TO_LPN(stream, raidLPA);
            
            if(ppc_cache->Exists(stream, raidLPA)){
                Data_Cache_Slot_Type slot = ppc_cache->Get_slot(stream, raidLPA);
				sim_time_type timestamp = slot.Timestamp;
				NVM::memory_content_type content = slot.Content;
				if (tr->DataTimeStamp > timestamp) {
					timestamp = tr->DataTimeStamp;
					content = tr->Content;
				}
				ppc_cache->Update_data(stream, raidLPA, content, timestamp, tr->write_sectors_bitmap | slot.State_bitmap_of_existing_sectors);
            }else{
                if (!ppc_cache->Check_free_slot_availability()) {
					Data_Cache_Slot_Type evicted_slot = ppc_cache->Evict_one_slot_lru();
					if (evicted_slot.Status == Cache_Slot_Status::DIRTY_NO_FLASH_WRITEBACK) {
                        std::vector<NVM_Transaction_Flash*> trans = create_evict_trans(evicted_slot, user_request, nullptr);
                        dynamic_cast<Address_Mapping_Unit_Page_Level_And_RAID*>(static_cast<FTL*>(nvm_firmware)->Address_Mapping_Unit)->Semilate_page(evicted_slot.streamid, evicted_slot.LPA);
                        evicted_cache_slots->emplace_back(trans[0]);
                        for(int i = 1; i < trans.size(); ++i){
                            readTrans.push_back(trans[i]);
                        }
						cache_eviction_read_size_in_sectors += count_sector_no_from_status_bitmap(evicted_slot.State_bitmap_of_existing_sectors);
                    }
				}
				ppc_cache->Insert_write_data(stream, raidLPA, tr->Content, tr->DataTimeStamp, tr->write_sectors_bitmap);
            }
            dram_write_size_in_sectors += count_sector_no_from_status_bitmap(tr->write_sectors_bitmap);
            it++;
        }
    }
    user_request->Sectors_serviced_from_cache += dram_write_size_in_sectors;

    if (dram_write_size_in_sectors) {
        Memory_Transfer_Info* write_transfer_info = new Memory_Transfer_Info;
        write_transfer_info->Size_in_bytes = dram_write_size_in_sectors * SECTOR_SIZE_IN_BYTE;
        write_transfer_info->Related_request = user_request;
        write_transfer_info->next_event_type = Data_Cache_Simulation_Event_Type::MEMORY_WRITE_FOR_USERIO_FINISHED;
        write_transfer_info->Stream_id = user_request->Stream_id;
        service_dram_access_request(write_transfer_info);
    }

    if(evicted_cache_slots->size() > 0){
        
        Memory_Transfer_Info* read_transfer_info = new Memory_Transfer_Info;
        read_transfer_info->Size_in_bytes = cache_eviction_read_size_in_sectors * SECTOR_SIZE_IN_BYTE;
        read_transfer_info->Related_request = evicted_cache_slots;
        read_transfer_info->next_event_type = Data_Cache_Simulation_Event_Type::MEMORY_READ_FOR_CACHE_EVICTION_FINISHED;
        read_transfer_info->Stream_id = user_request->Stream_id;
        
        service_dram_access_request(read_transfer_info);
    }
    //std::cout << "insert " << user_request->Transaction_list.size() << std::endl;
    /*in ppc the cache do not hold actual data*/
    for(auto readTran : readTrans){
        // std::cout << readTran << std::endl;
        user_request->Transaction_list.push_back(readTran);
    }
    static_cast<FTL*>(nvm_firmware)->Address_Mapping_Unit->Translate_lpa_to_ppa_and_dispatch(user_request->Transaction_list);
    if(evicted_cache_slots->size() > 0){
        for(auto &evicted_cache_slot : *evicted_cache_slots){
            user_request->Transaction_list.push_back(evicted_cache_slot);
        }
    }else{
        delete evicted_cache_slots;
    }
    
}

void Data_Cache_Manager_Flash_RAID::handle_transaction_serviced_signal_from_PHY(NVM_Transaction_Flash* transaction){
    //First check if the transaction source is a user request or the cache itself
    if (transaction->Source != Transaction_Source_Type::USERIO && transaction->Source != Transaction_Source_Type::CACHE) {
        return;
    }
    
    if (transaction->Source == Transaction_Source_Type::USERIO) {
        _my_instance->broadcast_user_memory_transaction_serviced_signal(transaction);
    }
    if (transaction->Type == Transaction_Type::READ) {
        if (((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite != NULL) {
            if(transaction->relatedCount == nullptr){
                ((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->RelatedRead = NULL;
            }else{
                if(transaction->UserIORequest)
                    transaction->UserIORequest->Transaction_list.remove(transaction);

                if(--(*transaction->relatedCount) == 0){
                    // std::cout << 0 << std::endl;
                    // ((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->RelatedRead = NULL;
                    // delete transaction->relatedCount;
                }
            }
            return;
        }
    }
    if(transaction->Source == Transaction_Source_Type::USERIO){
        if (transaction->Type == Transaction_Type::READ) {
            if (((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite != NULL) {
                ((NVM_Transaction_Flash_RD*)transaction)->RelatedWrite->RelatedRead = NULL;
                return;
            }
        }
    }else{
        if (((Data_Cache_Manager_Flash_RAID*)_my_instance)->ppc_cache->Exists(transaction->Stream_id, ((NVM_Transaction_Flash_WR*)transaction)->LPA)) {
            Data_Cache_Slot_Type slot = ((Data_Cache_Manager_Flash_RAID*)_my_instance)->ppc_cache->Get_slot(transaction->Stream_id, ((NVM_Transaction_Flash_WR*)transaction)->LPA);
            sim_time_type timestamp = slot.Timestamp;
            NVM::memory_content_type content = slot.Content;
            if (((NVM_Transaction_Flash_WR*)transaction)->DataTimeStamp >= timestamp) {
                ((Data_Cache_Manager_Flash_RAID*)_my_instance)->ppc_cache->Remove_slot(transaction->Stream_id, ((NVM_Transaction_Flash_WR*)transaction)->LPA);
            }
        }
    }
    if(transaction->UserIORequest){
        transaction->UserIORequest->Transaction_list.remove(transaction);
    }
    //std::cout << transaction->UserIORequest->Transaction_list.size() << std::endl;
    if (transaction->UserIORequest && _my_instance->is_user_request_finished(transaction->UserIORequest)) {
        _my_instance->broadcast_user_request_serviced_signal(transaction->UserIORequest);
    }
}

void Data_Cache_Manager_Flash_RAID::Setup_triggers(){
    Data_Cache_Manager_Base::Setup_triggers();
    flash_controller->ConnectToTransactionServicedSignal(handle_transaction_serviced_signal_from_PHY);
}

void Data_Cache_Manager_Flash_RAID::Do_warmup(std::vector<Utils::Workload_Statistics*> workload_stats){
    double total_write_arrival_rate = 0, total_read_arrival_rate = 0;
    switch (sharing_mode) {
        case Cache_Sharing_Mode::SHARED:
            //Estimate read arrival and write arrival rate
            //Estimate the queue length based on the arrival rate
            for (auto &stat : workload_stats) {
                switch (caching_mode_per_input_stream[stat->Stream_id]) {
                    case Caching_Mode::TURNED_OFF:
                        break;
                    case Caching_Mode::READ_CACHE:
                        if (stat->Type == Utils::Workload_Type::SYNTHETIC) {
                        } else {
                        }
                        break;
                    case Caching_Mode::WRITE_CACHE:
                        if (stat->Type == Utils::Workload_Type::SYNTHETIC) {
                            unsigned int total_pages_accessed = 1;
                            switch (stat->Address_distribution_type)
                            {
                            case Utils::Address_Distribution_Type::STREAMING:
                                break;
                            case Utils::Address_Distribution_Type::RANDOM_HOTCOLD:
                                break;
                            case Utils::Address_Distribution_Type::RANDOM_UNIFORM:
                                break;
                            default:
                                break;
                            }
                        } else {
                        }
                        break;
                    case Caching_Mode::WRITE_READ_CACHE:
                        //Put items on cache based on the accessed addresses
                        if (stat->Type == Utils::Workload_Type::SYNTHETIC) {
                        } else {
                        }
                        break;
                }
            }
            break;
        case Cache_Sharing_Mode::EQUAL_PARTITIONING:
            for (auto &stat : workload_stats) {
                switch (caching_mode_per_input_stream[stat->Stream_id])
                {
                    case Caching_Mode::TURNED_OFF:
                        break;
                    case Caching_Mode::READ_CACHE:
                        //Put items on cache based on the accessed addresses
                        if (stat->Type == Utils::Workload_Type::SYNTHETIC) {
                        } else {
                        }
                        break;
                    case Caching_Mode::WRITE_CACHE:
                        //Estimate the request arrival rate
                        //Estimate the request service rate
                        //Estimate the average size of requests in the cache
                        //Fillup the cache space based on accessed adddresses to the estimated average cache size
                        if (stat->Type == Utils::Workload_Type::SYNTHETIC) {
                            //Estimate average write service rate
                            unsigned int total_pages_accessed = 1;
                            /*double average_write_arrival_rate, stdev_write_arrival_rate;
                            double average_read_arrival_rate, stdev_read_arrival_rate;
                            double average_write_service_time, average_read_service_time;*/
                            switch (stat->Address_distribution_type)
                            {
                                case Utils::Address_Distribution_Type::STREAMING:
                                    break;
                                case Utils::Address_Distribution_Type::RANDOM_HOTCOLD:
                                    break;
                                case Utils::Address_Distribution_Type::RANDOM_UNIFORM:
                                    break;
                                default:
                                    break;
                            }
                        } else {
                        }
                        break;
                    case Caching_Mode::WRITE_READ_CACHE:
                        //Put items on cache based on the accessed addresses
                        if (stat->Type == Utils::Workload_Type::SYNTHETIC) {
                        } else {
                        }
                        break;
                }
            }
            break;
    }
}

}