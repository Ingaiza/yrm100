#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/uhf_functions.hpp"
#include <optional>
#include <iomanip>

uint8_t* single_poll()
{
    // Create a UHFSerial instance
    UHFSerial* serial = new UHFSerial("/dev/ttyUSB0");

    if(serial == NULL) 
    {
        delete serial;
        return 0;
    }

    // Allocate memory for the UHF module
    M100Module* module = m100_module_alloc(serial);
    if(module == NULL) 
    {
        // Clean up: Free allocated memory
        serial->stop();
        delete serial;
        return 0;
    }

    // Create a UHFTag object to store the tag information
    UHFTag* uhf_tag = uhf_tag_alloc();
    if(uhf_tag == NULL) 
    {
        // Clean up: Free allocated memory
        m100_module_free(module);
        serial->stop();
        delete serial;
        return 0;
    }

    // Perform a single tag polling
    M100ResponseType poll_response_ = m100_single_poll(module, uhf_tag);

    // Check the poll_response_ type and handle accordingly
    if(poll_response_ == M100SuccessResponse) {
        
        std::cout<<"Single Poll response success"<<std::endl;        
        // Retrieve and display EPC, PC, and CRC information
        uint8_t* epc_data = uhf_tag_get_epc(uhf_tag);
        assert(epc_data != nullptr);
        std::cout<<"epc_data assertion passed"<<std::endl;
        size_t epc_size = uhf_tag_get_epc_size(uhf_tag);
        uint16_t pc = uhf_tag_get_epc_pc(uhf_tag);
        uint16_t crc = uhf_tag_get_epc_crc(uhf_tag);

        uint8_t* return_epc = new uint8_t[epc_size];
        std::memcpy(return_epc, epc_data, epc_size);

        printf("EPC: ");
        for(size_t i = 0; i < epc_size; i++) {
            printf("%02X ", epc_data[i]);
        }
        printf("\nPC: %04X\nCRC: %04X\n", pc, crc);

        // Clean up: Free allocated memory
        assert(uhf_tag != nullptr);
        std::cout<<"uhf_tag assertion passed"<<std::endl;
        uhf_tag_free(uhf_tag);
        assert(module != nullptr);
        std::cout<<"module assertion passed"<<std::endl;
        m100_module_free(module);
        assert(serial != nullptr);
        std::cout<<"serial assertion passed"<<std::endl;
        serial->stop();
        delete serial;
        std::cout<<"ROS2 Single Poll function completed"<<std::endl;

        return return_epc;
    } 
    else 
    {
        std::cout<<"Single Poll response failed"<<std::endl;        
        // Clean up:Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        delete serial;
        return 0;
    }

}
// uint8_t* read(uint8_t* read_data)
// {
// }
uint8_t* read_select(uint8_t* read_data)
{
    UHFSerial* serial = new UHFSerial("/dev/ttyUSB0");
    DEFAULT_PARAMS_ params ;
    if(serial == NULL) 
    {
        delete serial;
        return 0;
    }

    // Allocate memory for the UHF module
    M100Module* module = m100_module_alloc(serial);
    if(module == NULL) 
    {
        delete serial;
        return 0;
    }

    // Create a UHFTag object to store the tag information
    UHFTag* uhf_tag = uhf_tag_alloc();
    if(uhf_tag == NULL) 
    {
        // Clean up: Free allocated memory
        m100_module_free(module);
        serial->stop();
        delete serial;
        return 0;
    }

    // Perform a single tag polling
    M100ResponseType poll_response_ = m100_single_poll(module, uhf_tag);

    // Check the poll_response_ type and handle accordingly
    if(poll_response_ != M100SuccessResponse) 
    {
        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        delete serial;
        return 0;  
    } 

    M100ResponseType set_select_response_;

    if(poll_response_ == M100SuccessResponse)
    {
        set_select_response_ = m100_set_select(module , uhf_tag);
    }

    if(set_select_response_ != M100SuccessResponse)
    {
        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        delete serial;
        return 0;
    }
  

    M100ResponseType read_tag_response_;

    if(set_select_response_ == M100SuccessResponse)
    {
        read_tag_response_ = m100_read_label_data_storage(module,uhf_tag,UserBank,params.ACCESS_PSWD_,params.WORD_COUNT_);
    }

    if(read_tag_response_ == M100SuccessResponse)
    {
        assert(read_data != nullptr);
        std::cout<<"read_data assertion passed"<<std::endl;
        std::vector<uint8_t> data = uhf_tag_get_user(uhf_tag);
        assert(!data.empty());
        std::cout<<"data assertion passed"<<std::endl;
        size_t length = uhf_tag_get_user_size(uhf_tag);
        // std::memcpy(read_data,data,length);
        std::copy(data.begin(), data.end(), read_data);
        uint8_t* epc_data = uhf_tag_get_epc(uhf_tag);
        assert(epc_data != nullptr);
        std::cout<<"epc_data assertion passed"<<std::endl;
        size_t epc_size = uhf_tag_get_epc_size(uhf_tag);
        uint16_t pc = uhf_tag_get_epc_pc(uhf_tag);
        uint16_t crc = uhf_tag_get_epc_crc(uhf_tag);

        uint8_t* return_epc = new uint8_t[epc_size];
        std::memcpy(return_epc, epc_data, epc_size);
        
        printf("READ EPC: ");
        for(size_t i = 0; i < epc_size; i++) {
            printf("%02X ", epc_data[i]);
        }
        printf("\nPC: %04X\nCRC: %04X\n", pc, crc);

        printf("READ DATA: ");
        for(size_t i = 0; i < length; i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");
        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        delete serial;
        return return_epc;
    }    
    else 
    {   
        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        delete serial;
        return 0;
    }
}

uint8_t* write_tag(std::vector<uint8_t> write_data)
{
    UHFSerial* serial = new UHFSerial("/dev/ttyUSB0");
    DEFAULT_PARAMS_ PARAMS_ ;
    if(serial == NULL) 
    {
        return 0;
    }
    // Allocate memory for the UHF module
    M100Module* module = m100_module_alloc(serial);
    if(module == NULL) 
    {
        // Clean up: Free allocated memory
        serial->stop();
        return 0;
    }

    // Create a UHFTag object to store the tag information
    UHFTag* uhf_tag = uhf_tag_alloc();
    if(uhf_tag == NULL)
    {
        // Clean up: Free allocated memory
        m100_module_free(module);
        serial->stop();
        return 0;
    }

    // Perform a single tag polling
    M100ResponseType poll_response_ = m100_single_poll(module, uhf_tag);

    // Check the poll_response_ type and handle accordingly
    if(poll_response_ != M100SuccessResponse) 
    {
        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        return 0;
    }
    M100ResponseType set_select_response_;

    if(poll_response_ == M100SuccessResponse)
    {
        set_select_response_ = m100_set_select(module , uhf_tag);
    }

    if(set_select_response_ != M100SuccessResponse)
    {
        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        return 0;
    }

    // save uhf_tag data
    UHFTag* saved_tag_ = uhf_tag;
    std::vector<uint8_t> user_data_;
    user_data_.resize(16);
    size_t user_data_size_;
    user_data_size_ = sizeof(write_data);
    std::cout<<"Starting memcpy .....\n";
    // printf("Starting memcpy....");
    // std::memcpy(user_data_,write_data,user_data_size_);
    std::copy(write_data.begin(),write_data.end(),user_data_.begin());
    // set user data in saved_tag_
    // printf("setting user....");
    std::cout<<"Setting user .....\n";
    uhf_tag_set_user(saved_tag_,user_data_,user_data_size_);
    uhf_tag_set_user_size(saved_tag_,user_data_size_);


    M100ResponseType write_tag_response_;

    if(set_select_response_ == M100SuccessResponse)
    {
        // printf("Starting write....");
        std::cout<<"Starting write .....\n";
        write_tag_response_ = m100_write_label_data_storage(module,saved_tag_,uhf_tag,UserBank,PARAMS_.SOURCE_ADDR_,PARAMS_.ACCESS_PSWD_);
        std::cout<<"write complete .....\n";
        // printf("write complete....");

    }

    if(write_tag_response_ == M100SuccessResponse)
    {
        uint8_t* epc_data = uhf_tag_get_epc(saved_tag_);
        size_t epc_size = uhf_tag_get_epc_size(saved_tag_);

        uint8_t* return_epc = new uint8_t[epc_size];
        std::memcpy(return_epc, epc_data, epc_size);

        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        return return_epc;
    }    
    else if(write_tag_response_ == M100ValidationFail)
    {
        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        return 0;
    }
    else if(write_tag_response_ == M100NoTagResponse)
    {
        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        return 0;
    }
    else
    {
        // Clean up: Free allocated memory
        uhf_tag_free(uhf_tag);
        m100_module_free(module);
        serial->stop();
        return 0;
    }


    // Clean up: Free allocated memory
    uhf_tag_free(uhf_tag);
    m100_module_free(module);
    serial->stop();
    return 0;
}

std::optional<std::vector<uint8_t>> multi_poll()
{
    UHFSerial* serial = new UHFSerial("/dev/ttyUSB0");
    if(serial == NULL) 
    {
        return std::nullopt;
    }

    // Allocate memory for the UHF module
    M100Module* module = m100_module_alloc(serial);
    if(module == NULL) 
    {
        delete serial;
        return std::nullopt;
    }

    // Create a UHFTag object to store the tag information
    UHFTag* uhf_tag[5];

    // Perform a single tag polling
    M100ResponseType poll_response_ = m100_multi_poll(module);
    std::cout<<"Completed m100 multi poll"<<std::endl;
    std::vector<uint8_t> epc_data;
    size_t epc_size;
    size_t EPC = 12;

    // Check the poll_response_ type and handle accordingly
    if(poll_response_ == M100SuccessResponse) 
    {
        std::cout<<"TAG NO: "<<module->tag_no<<std::endl;
        for(int alloc=0;alloc<module->tag_no;alloc++)
        {
            std::cout<<"Completed allocation for uhf_tag["<<alloc<<"]"<<std::endl;
            uhf_tag[alloc] = uhf_tag_alloc();
        }
        std::cout<<"Completed all tags allocation"<<std::endl;
        M100ResponseType alloc_response_ = multi_poll_tag_alloc(module, uhf_tag);
        std::cout<<"Completed multi poll tag alloc function"<<std::endl;
        if(alloc_response_ == M100SuccessResponse)
        {   
            epc_size = module->tag_no * EPC;
            epc_data.resize(epc_size);
            int value = epc_size;
            std::cout<<"TOTAL EPC SIZE IS TAG NO("<<module->tag_no<<") * EPC of one tag(12) = "<<value<<std::endl;
            for (int i = 0; i < module->tag_no; i++)
            {
                // Retrieve and display EPC, PC, and CRC information
                uint8_t* epc = uhf_tag_get_epc(uhf_tag[i]);

                std::copy(epc, epc + EPC, epc_data.begin() + i*EPC);
            }
        }
        std::cout<<"Loading to epc_data vector complete"<<std::endl;

        // uint8_t* return_epc = new uint8_t[epc_size];
        // std::copy(epc_data.begin(),epc_data.end(),return_epc);       
        
        // *return_epc_size = epc_size;
        // Check if epc_size is valid
        // if (epc_size == 0) 
        // {
        //     std::cerr << "Invalid epc_size: " << epc_size << std::endl;
        //     return std::nullopt;
        // }

        // try 
        // {               
        //     // Verify epc_data has correct size
        //     if (epc_data.size() != epc_size) 
        //     {
        //         std::cerr <<"Size mismatch: epc_data size: "<< epc_data.size()<< ", epc_size: "<< epc_size <<std::endl;
        //         return std::nullopt;
        //     }

        // } 
        // catch (const std::bad_alloc& e) 
        // {
        //     std::cerr << "Memory allocation failed: " << e.what() << std::endl;
        //     return std::nullopt;
        // }

        // std::cout<<"Starting m100_stop_multi_poll"<<std::endl;
        M100ResponseType stop_response_ = m100_stop_multi_poll(module);

        // Clean up: Free allocated memory
        std::cout<<"Freeing uhf_tag Memory"<<std::endl;
        for(int free=0;free<module->tag_no;free++)
        {
            uhf_tag_free(uhf_tag[free]);
        }
        std::cout<<"Freeing module Memory"<<std::endl;
        m100_module_free(module);
        std::cout<<"Stoping Boost ASIO serial communication "<<std::endl;
        serial->stop();
        delete serial;
        return epc_data;

    } 
    else 
    {
        // Clean up: Free allocated memory
        std::cout<<"Freeing module Memory"<<std::endl;
        m100_module_free(module);
        std::cout<<"Stoping Boost ASIO serial communication "<<std::endl;
        serial->stop();
        delete serial;
        return std::nullopt;
    }
}