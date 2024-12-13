#include "/home/aimbot/yrm100/src/yrm100/include/yrm100/uhf_serial.hpp"
#include <iostream>

UHFSerial::UHFSerial(const std::string& port_name)
    : baudrate_(DEFAULT_BAUDRATE),
      tick_(WAIT_TICK),
      running_(false) {
    
    buffer_ = uhf_buffer_alloc(RX_BUFFER_SIZE);
    if (!buffer_) {
        throw std::runtime_error("Failed to allocate buffer");
    }

    serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_);
    serial_port_->open(port_name);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
}

UHFSerial::~UHFSerial() {
    stop();
    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }
    if (buffer_) {
        uhf_buffer_free(buffer_);
    }
}

void UHFSerial::rx_worker() {
    try {
        while (running_) {
            io_context_.run();
            // If io_context runs out of work, reset it
            io_context_.restart();
        }
    }
    catch (const std::exception& e) {
        std::cerr << "RX worker exception: " << e.what() << std::endl;
        running_ = false;
    }
}

void UHFSerial::start() {
    if (!running_) {
        running_ = true;
        uhf_buffer_reset(buffer_);
        
        // Start the async receive operation
        start_receive();
        
        // Start the worker thread
        worker_thread_ = std::make_unique<std::thread>([this]() { rx_worker(); });
    }
}

void UHFSerial::stop() {
    if (running_) {
        running_ = false;
        
        // Cancel any pending operations
        if (serial_port_ && serial_port_->is_open()) {
            boost::system::error_code ec;
            serial_port_->cancel(ec);
        }
        
        // Stop the io_context
        io_context_.stop();
        
        // Wait for the worker thread to finish
        if (worker_thread_ && worker_thread_->joinable()) {
            worker_thread_->join();
        }
    }
}

void UHFSerial::start_receive() {
    if (!running_ || !serial_port_ || !serial_port_->is_open()) return;
    
    serial_port_->async_read_some(
        boost::asio::buffer(read_buffer_, 1),
        [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
            handle_receive(error, bytes_transferred);
        });
}

void UHFSerial::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (error) {
        if (error != boost::asio::error::operation_aborted) {
            std::cerr << "Read error: " << error.message() << std::endl;
        }
        return;
    }

    if (bytes_transferred > 0) {
        if (!uhf_is_buffer_closed(buffer_)) {
            uhf_buffer_append_single(buffer_, read_buffer_[0]);
            tick_reset();

            if (read_buffer_[0] == FRAME_END) {
                uhf_buffer_close(buffer_);
                std::cout << "UHF Total length read = " << uhf_buffer_get_size(buffer_) << std::endl;
                
                if (callback_) {
                    callback_(uhf_buffer_get_data(buffer_), callback_ctx_);
                }
                
                // Reset buffer for next frame
                uhf_buffer_reset(buffer_);
            }
        }
    }

    // Continue receiving
    start_receive();
}

void UHFSerial::send(const uint8_t* data, size_t size) {
    if (!serial_port_->is_open()) return;
    
    boost::asio::async_write(
        *serial_port_,
        boost::asio::buffer(data, size),
        [](const boost::system::error_code& error, std::size_t /*bytes_transferred*/) {
            if (error) {
                std::cerr << "Write error: " << error.message() << std::endl;
            }
        });
}

// void UHFSerial::send_wait(const uint8_t* data, size_t size) {
//     if (!serial_port_->is_open()) return;
    
//     boost::system::error_code error;
//     boost::asio::write(*serial_port_, boost::asio::buffer(data, size), error);
//     if (error) {
//         std::cerr << "Write error: " << error.message() << std::endl;
//     }
// }
size_t UHFSerial::send_wait(const uint8_t* data, size_t size, uint8_t* response_framed_) 
{
    assert(data != nullptr);
    assert(response_framed_ != nullptr);

    if (!serial_port_->is_open()) return 0;

    boost::system::error_code error;
    boost::asio::write(*serial_port_, boost::asio::buffer(data, size), error);
    if (error) 
    {
        std::cerr << "Write error: " << error.message() << std::endl;
        return 0;
    }

    // Wait for response_framed_
    size_t received = 0;
    size_t buffer_framed_size_ = 0;
    uint8_t buffer[256];
    uint8_t buffer_framed_[256];
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5); // Set a 5-second timeout
    bool start_frame_ = false;
    bool end_frame_ = false;

    while (true) 
    {
        size_t bytes_read = serial_port_->read_some(boost::asio::buffer(buffer + received, sizeof(buffer) - received), error);
        if (error) 
        {
            std::cerr << "Read error: " << error.message() << std::endl;
            return buffer_framed_size_;
        }

        received += bytes_read;
        if (buffer[received-1] == 0x7E)
        {
            int b = 0;
            for (size_t f = 0; f < received && !end_frame_; f++) 
            {
                if (buffer[f] == 0xBB && !start_frame_) 
                {
                    start_frame_ = true;
                    buffer_framed_[b++] = buffer[f];
                    buffer_framed_size_++;
                } 
                else if (start_frame_) 
                {
                    buffer_framed_[b++] = buffer[f];
                    buffer_framed_size_++;
                    if (buffer[f] == 0x7E) 
                    {
                        end_frame_ = true;
                        break;
                    }
                }
            }
        }
        
        if (end_frame_) 
        {
            break;
        }

        if (std::chrono::steady_clock::now() - start_time > timeout) 
        {
            std::cerr << "Timeout waiting for response" << std::endl;
            return buffer_framed_size_;
        }
    }

    std::memcpy(response_framed_, buffer_framed_, buffer_framed_size_);
    std::cout<<"Send Wait Completed"<<std::endl;
    return buffer_framed_size_;
}

void UHFSerial::send_wait_multi(Buffer** buffer_return_,const uint8_t* data, size_t size) 
{
    assert(data != nullptr);
    assert(buffer_return_ != nullptr);
    for(int i=0;i<5;i++)
    {
        assert(buffer_return_[i]->data != nullptr);
    }

    // if (!serial_port_->is_open()); // return M100SerialPortFailed;

    boost::system::error_code error;
    boost::asio::write(*serial_port_, boost::asio::buffer(data, size), error);
    if (error) 
    {
        std::cerr << "Write error: " << error.message() << std::endl;
        // return M100SerialPortFailed;
    }

    // Wait for response_framed_
    size_t received = 0;
    size_t buffer_framed_size_ = 0;
    uint8_t buffer[512];
    uint8_t buffer_framed_[256];
    Buffer* buffer_loaded_[5];
    
    for (int i = 0; i < 5; i++)  
    {
        // std::cout << "Starting buffer_loaded_[] allocation for index " << i << std::endl;
        // Allocate the Buffer struct itself
        buffer_loaded_[i] = (Buffer*)malloc(sizeof(Buffer));
        if (buffer_loaded_[i] == nullptr) 
        {
            std::cerr << "Failed to allocate Buffer struct for index " << i << std::endl;
            // Clean up previously allocated buffers
            for (int j = 0; j < i; j++) 
            {
                if (buffer_loaded_[j] != nullptr) 
                {
                    if (buffer_loaded_[j]->data != nullptr) 
                    {
                        free(buffer_loaded_[j]->data);
                    }
                    free(buffer_loaded_[j]);
                }
            }
        }

        // Allocate the data buffer
        buffer_loaded_[i]->data = (uint8_t*)malloc(256);
        if (buffer_loaded_[i]->data == nullptr) 
        {
            std::cerr << "Failed to allocate data buffer for index " << i << std::endl;
            free(buffer_loaded_[i]);
            // Clean up previously allocated buffers
            for (int j = 0; j < i; j++) 
            {
                if (buffer_loaded_[j] != nullptr) 
                {
                    if (buffer_loaded_[j]->data != nullptr) 
                    {
                        free(buffer_loaded_[j]->data);
                    }
                    free(buffer_loaded_[j]);
                }
            }
        }

        // Initialize other fields
        buffer_loaded_[i]->capacity = 256;
        buffer_loaded_[i]->size = 0;
        buffer_loaded_[i]->head = 0;
        buffer_loaded_[i]->tail = 0;
        buffer_loaded_[i]->closed = false;
        buffer_loaded_[i]->loaded = false;
        // std::cout << "Completed allocation for buffer_loaded_[" << i << "]" << std::endl;
    }
    assert(buffer_loaded_ != nullptr);
    for(int i=0;i<5;i++)
    {
        assert(buffer_loaded_[i]->data != nullptr);
    }
    assert(buffer_framed_ != nullptr);

    // int tags_ = 0;

    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5); // Set a 5-second timeout
    bool read_ = true;
    bool start_frame_ = false;
    bool end_frame_ = false;

    while (read_) 
    {
        size_t bytes_read = serial_port_->read_some(boost::asio::buffer(buffer + received, sizeof(buffer) - received), error);
        if (error) 
        {
            std::cerr << "Read error: " << error.message() << std::endl;
            // return M100SerialPortFailed;
        }

        received += bytes_read;

        if (std::chrono::steady_clock::now() - start_time > timeout) 
        {
            read_ = false;
        }
    }
    std::cout<<"Received Bytes: "<<received<<std::endl;
    
    uint8_t crc_buffer[5];
    bool load_tag = true;
    for (int frame = 0,index = 0;frame < 1 && index < 5;frame++)
    {
        int start_offset_ = 0;
        int b = 0;
        for (size_t f = 0;f < received && !end_frame_; f++) 
        {
            if (buffer[f] == 0xBB && !start_frame_) 
            {
                start_frame_ = true;
                buffer_framed_[b++] = buffer[f];
                buffer_framed_size_++;
            } 
            else if (start_frame_) 
            {
                buffer_framed_[b++] = buffer[f];
                buffer_framed_size_++;
                if (buffer[f] == 0x7E) 
                {
                    end_frame_ = true;
                    break;
                }
            }
            else if(buffer[f] != 0xBB && !start_frame_)
            {
                start_offset_+= 1;
            }
        }
        if(end_frame_)
        {
            // printf("Complete Received Buffer:\n");
            // for(size_t i = 0; i < received; i++) 
            // {
            //     printf("%02X ", buffer[i]);
            // }
            // printf("\n");
            // printf("Bytes Popped from Buffer:\n");
            for(size_t pop = 0;pop<(buffer_framed_size_ + start_offset_);pop++)
            {
                pop_from_buffer(buffer,received);
                // printf("%02X ",buffer[0]);
            }
            // printf("\n");
            printf("buffer_framed:\n");
            for(size_t i = 0; i < buffer_framed_size_; i++) 
            {
                printf("%02X ", buffer_framed_[i]);
            }
            printf("\n");
            uint8_t crc = 0;
            crc = buffer_framed_[buffer_framed_size_- 4];
            printf("CRC: %02X \n", crc);
            printf("Index: %d \n",index);
            if(index == 0)
            {
                crc_buffer[index] = crc;
                std::memcpy(buffer_loaded_[index]->data,buffer_framed_,buffer_framed_size_);
                buffer_loaded_[index]->size = buffer_framed_size_;
                buffer_loaded_[index]->loaded = true;
                printf("{IF}Loading to buffer_loaded_[%d]\n",index);
                index++;
            }
            else
            {
                for(int i=0;i<index;i++)
                {
                    if(crc == crc_buffer[i])
                    {
                        load_tag = false;
                        printf("CRC MATCH\n");
                    }
                }
                if(load_tag)
                {
                    crc_buffer[index] = crc;
                    std::memcpy(buffer_loaded_[index]->data,buffer_framed_,buffer_framed_size_);
                    buffer_loaded_[index]->size = buffer_framed_size_;
                    buffer_loaded_[index]->loaded = true;
                    printf("{ELSE}Loading to buffer_loaded_[%d]\n",index);
                    index++;  
                }
            }
            // save the extracted command frame
            // printf("Index: %d\n", index);
            
            
            //reset the buffer_framed_size_
            buffer_framed_size_ = 0;

            if(buffer[0] == 0xBB)
            {   
                printf("Found another tag\n");
                printf("Next tag will be loaded at Index: %d\n\n",index);
                if(index < 5)
                {
                    if(buffer_loaded_[index]->loaded) std::cout<<"loaded status of next tag of index"<<index<<"is: True"<<std::endl;
                }
                start_frame_ = false;
                end_frame_ = false;
                load_tag = true;
                frame-= 1;
            }
            else
            {
                printf("%02X",buffer[0]);
                printf("No other tag was found\n");
            }

            // if(index == 5) break;

        }
    }

    std::cout<<"Starting data loading"<<std::endl;
    for(int load=0;load<5;load++)
    {   
        if(buffer_loaded_[load]->loaded)
        {
            // printf("load index: %d\n",load);
            std::memcpy(buffer_return_[load]->data,buffer_loaded_[load]->data,buffer_loaded_[load]->size);
            buffer_return_[load]->size = buffer_loaded_[load]->size;
            buffer_return_[load]->loaded = true;

        } 
        // free(buffer_loaded_[load]->data);
        free(buffer_loaded_[load]); 
    }
    // std::memcpy(response_framed_, buffer_framed_, buffer_framed_size_);
    std::cout<<"Data loading complete"<<std::endl;
    
}

void UHFSerial::set_receive_callback(receive_callback_t callback, void* ctx) {
    callback_ = callback;
    callback_ctx_ = ctx;
}

void UHFSerial::set_baudrate(uint32_t baudrate) {
    baudrate_ = baudrate;
    if (serial_port_->is_open()) {
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    }
}

bool UHFSerial::tick() {
    if (tick_ > 0) {
        --tick_;
    }
    return tick_ == 0;
}

void UHFSerial::tick_reset() {
    tick_ = WAIT_TICK;
}

void UHFSerial::pop_from_buffer(uint8_t* buffer, size_t& buffer_size) 
{
    if (buffer_size == 0) {
     std::cout<<"Passed buffer is Empty"<<std::endl;
    }    
    // Shift the remaining elements to the left
    for (size_t i = 1; i < buffer_size; i++) {
        buffer[i-1] = buffer[i];
    }

    buffer_size--;
}