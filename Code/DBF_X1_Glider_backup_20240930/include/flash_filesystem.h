#ifndef FLASH_FILESYSTEM
#define FLASH_FILESYSTEM

void init_fs();
void create_dir(const char* dir_path);
unsigned long list_files(const char* dir_path);
unsigned int read_file(const char* path, const char* file_contents="", bool serial_output=true, bool var_output=false, int read_delay=0, bool read_complete=true);
unsigned int write_file(const char* path, const char* message, bool add_newline=false);
unsigned int append_file(const char* path, const char* message, bool add_newline=false);
void delete_file(const char* path);
void delete_dir(const char* dir_path);

#endif