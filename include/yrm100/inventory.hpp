#pragma once

#include <cstring>
#include <ctime>
#include <vector>
#include <stdexcept>
#include <cstdint>

class RFIDDataMemory {
private:
    void writeUint16(size_t offset, uint16_t value);

    uint16_t readUint16(size_t offset) const;

public:
    std::vector<uint8_t> data;

    RFIDDataMemory();

    // Get the raw data
    const std::vector<uint8_t>& getData() const;

    // Set date (days since Jan 1, 2000)
    void setDate(const std::tm& date);

    // Get stored date
    std::tm getDate() const;

    void setProductCategory(uint16_t categoryId);

    uint16_t getProductCategory() const;

    void setLocation(uint16_t locationCode);

    uint16_t getLocation() const;

    void setStatus(bool inStock = true, bool reserved = false, 
                   bool damaged = false, bool expired = false);

    struct Status {
        bool inStock;
        bool reserved;
        bool damaged;
        bool expired;
    };

    Status getStatus() const
    {
        return {
            static_cast<bool>(data[6] & 0x01),
            static_cast<bool>(data[6] & 0x02),
            static_cast<bool>(data[6] & 0x04),
            static_cast<bool>(data[6] & 0x08)
        };
    };

    void setQuantity(uint8_t quantity);

    uint8_t getQuantity() const;

    void setPrice(uint16_t priceInCents);

    uint16_t getPrice() const;

    void setSupplierId(uint16_t supplierId);

    uint16_t getSupplierId() const;

    void setBatchNumber(uint16_t batchNum);

    uint16_t getBatchNumber() const;

    void calculateChecksum();

    bool verifyChecksum() const;
};