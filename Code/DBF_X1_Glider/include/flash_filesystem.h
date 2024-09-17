#ifndef FLASH_FILESYSTEM
#define FLASH_FILESYSTEM

void init_fs();
void read_file(const char* path);
void write_file(const char* path, const char* message);
void append_file(const char* path, const char* message);
void delete_file(const char* path);
void clear_directory(const char* path);

#endif