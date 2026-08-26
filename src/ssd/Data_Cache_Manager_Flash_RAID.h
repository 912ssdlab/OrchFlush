#ifndef DATA_CACHE_MANAGER_FLASH_RAID_H
#define DATA_CACHE_MANAGER_FLASH_RAID_H

#include <unordered_map>

#include "Data_Cache_Manager_Base.h"
#include "Data_Cache_Flash.h"


namespace SSD_Components
{

class Data_Cache_Manager_Flash_RAID : public Data_Cache_Manager_Base{
private:
    Data_Cache_Flash *ppc_cache;
    bool memory_channel_is_busy = false;
    std::queue<Memory_Transfer_Info*> dram_execution_queue;
    NVM_PHY_ONFI * flash_controller;

    unsigned int capacity_in_bytes, capacity_in_pages;
    unsigned int sector_no_per_page;
    int dram_execution_list_turn;
    unsigned long long gcCacheC = 0;
public:
    Data_Cache_Manager_Flash_RAID(const sim_object_id_type& id, Host_Interface_Base* host_interface, NVM_Firmware* firmware, NVM_PHY_ONFI* flash_controller,
			unsigned int total_capacity_in_bytes,
			unsigned int dram_row_size, unsigned int dram_data_rate, unsigned int dram_busrt_size, sim_time_type dram_tRCD, sim_time_type dram_tCL, sim_time_type dram_tRP,
			Caching_Mode* caching_mode_per_input_stream, Cache_Sharing_Mode sharing_mode, 
			unsigned int stream_count, unsigned int sector_no_per_page, unsigned int back_pressure_buffer_max_depth);
    ~Data_Cache_Manager_Flash_RAID();
    void Execute_simulator_event(MQSimEngine::Sim_Event* ev);
    void Do_warmup(std::vector<Utils::Workload_Statistics*> workload_stats);
    void Setup_triggers();
    bool is_exist(stream_id_type stream, LPA_type lpa);

    void gc_eviction(std::vector<std::pair<stream_id_type, LPA_type>> keys, NVM_Transaction_Flash_ER* gc_wl_erase_tr);
private:
    std::vector<NVM_Transaction_Flash*> create_evict_trans(const Data_Cache_Slot_Type evicted_slot, User_Request* user_request, NVM_Transaction_Flash_ER* gc_wl_erase_tr);
    void process_new_user_request(User_Request* user_request);
    void write_to_destage_buffer(User_Request* user_request);
    void service_dram_access_request(Memory_Transfer_Info* request_info);
    static void handle_transaction_serviced_signal_from_PHY(NVM_Transaction_Flash* transaction);
};

}

#endif