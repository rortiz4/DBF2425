#ifndef FLASH_FILESYSTEM
#define FLASH_FILESYSTEM

void init_fs();
void create_dir(const char* dir_path);
void list_files(const char* dir_path);
void read_file(const char* path);
void write_file(const char* path, const char* message, bool add_newline=true);
void append_file(const char* path, const char* message, bool add_newline=true);
void delete_file(const char* path);
void delete_dir(const char* dir_path);

#endif