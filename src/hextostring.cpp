#include "/home/ingaiza/yrm_module/src/yrm100/include/yrm100/hextostring.hpp"

std::string HexChunkProcessor::bytesToHexString(const std::vector<uint8_t>& bytes, size_t start, size_t length)
{
    std::stringstream ss;
    for (size_t i = start; i < start + length && i < bytes.size(); ++i) {
        ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
           << static_cast<int>(bytes[i]);
    }
    return ss.str();
}

std::vector<std::string> HexChunkProcessor::processHexChunks(std::vector<uint8_t>& data)
{   
    std::vector<std::string> chunks;
        size_t totalChunks = (data.size() + CHUNK_SIZE - 1) / CHUNK_SIZE; 
        chunks.reserve(totalChunks); 
        
        while (!data.empty()) {
            size_t chunkSize = std::min(CHUNK_SIZE, data.size());
            std::string hexString = bytesToHexString(data, 0, chunkSize);
            chunks.push_back(hexString);
            data.erase(data.begin(), data.begin() + chunkSize);
        }
        
        return chunks;
    
}

std::vector<std::string> HexChunkProcessor::processHexChunksNonDestructive(const std::vector<uint8_t>& data)
{
    std::vector<std::string> chunks;
    size_t totalChunks = (data.size() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    chunks.reserve(totalChunks);
    
    for (size_t offset = 0; offset < data.size(); offset += CHUNK_SIZE) {
        size_t chunkSize = std::min(CHUNK_SIZE, data.size() - offset);
        std::string hexString = bytesToHexString(data, offset, chunkSize);
        chunks.push_back(hexString);
    }
    
    return chunks;
    
}
