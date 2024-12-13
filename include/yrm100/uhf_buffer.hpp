#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <vector>

#define MAX_BUFFER_SIZE 200

typedef struct {
    uint8_t* data;
    size_t size;
    size_t capacity;
    size_t head;
    size_t tail;
    bool closed;
    bool loaded;
    bool multi;
} Buffer;

typedef struct {
    std::vector<uint8_t> vec_data;
    size_t size;
    size_t capacity;
    size_t head;
    size_t tail;
    bool closed;
    bool loaded = false;
} M_Buffer;

Buffer* uhf_buffer_alloc(size_t inital_capacity);
bool uhf_buffer_append_single(Buffer* buf, uint8_t value);
bool uhf_buffer_append(Buffer* buf, uint8_t* data, size_t size);
void uhf_buffer_append_byte(Buffer* buffer, uint8_t byte);


uint8_t* uhf_buffer_get_data(Buffer* buf);
size_t uhf_buffer_get_size(Buffer* buf);
bool uhf_is_buffer_closed(Buffer* buf);
void uhf_buffer_close(Buffer* buf);
void uhf_buffer_reset(Buffer* buf);
void uhf_buffer_free(Buffer* buf);