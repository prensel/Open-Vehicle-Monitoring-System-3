COMPONENT_OBJS += zlib/adler32.o
COMPONENT_OBJS += zlib/compress.o
COMPONENT_OBJS += zlib/crc32.o
COMPONENT_OBJS += zlib/deflate.o
COMPONENT_OBJS += zlib/gzclose.o
COMPONENT_OBJS += zlib/gzlib.o
COMPONENT_OBJS += zlib/gzread.o
COMPONENT_OBJS += zlib/gzwrite.o
COMPONENT_OBJS += zlib/infback.o
COMPONENT_OBJS += zlib/inffast.o
COMPONENT_OBJS += zlib/inflate.o
COMPONENT_OBJS += zlib/inftrees.o
COMPONENT_OBJS += zlib/trees.o
COMPONENT_OBJS += zlib/uncompr.o
COMPONENT_OBJS += zlib/zutil.o

COMPONENT_OBJS += libzip/lib/zip_add.o
COMPONENT_OBJS += libzip/lib/zip_add_dir.o
COMPONENT_OBJS += libzip/lib/zip_add_entry.o
COMPONENT_OBJS += libzip/lib/zip_algorithm_deflate.o
COMPONENT_OBJS += libzip/lib/zip_buffer.o
COMPONENT_OBJS += libzip/lib/zip_close.o
COMPONENT_OBJS += libzip/lib/zip_crypto_mbedtls.o
COMPONENT_OBJS += libzip/lib/zip_delete.o
COMPONENT_OBJS += libzip/lib/zip_dir_add.o
COMPONENT_OBJS += libzip/lib/zip_dirent.o
COMPONENT_OBJS += libzip/lib/zip_discard.o
COMPONENT_OBJS += libzip/lib/zip_entry.o
COMPONENT_OBJS += libzip/lib/zip_err_str.o
COMPONENT_OBJS += libzip/lib/zip_error.o
COMPONENT_OBJS += libzip/lib/zip_error_clear.o
COMPONENT_OBJS += libzip/lib/zip_error_get.o
COMPONENT_OBJS += libzip/lib/zip_error_get_sys_type.o
COMPONENT_OBJS += libzip/lib/zip_error_strerror.o
COMPONENT_OBJS += libzip/lib/zip_error_to_str.o
COMPONENT_OBJS += libzip/lib/zip_extra_field.o
COMPONENT_OBJS += libzip/lib/zip_extra_field_api.o
COMPONENT_OBJS += libzip/lib/zip_fclose.o
COMPONENT_OBJS += libzip/lib/zip_fdopen.o
COMPONENT_OBJS += libzip/lib/zip_file_add.o
COMPONENT_OBJS += libzip/lib/zip_file_error_clear.o
COMPONENT_OBJS += libzip/lib/zip_file_error_get.o
COMPONENT_OBJS += libzip/lib/zip_file_get_comment.o
COMPONENT_OBJS += libzip/lib/zip_file_get_external_attributes.o
COMPONENT_OBJS += libzip/lib/zip_file_get_offset.o
COMPONENT_OBJS += libzip/lib/zip_file_rename.o
COMPONENT_OBJS += libzip/lib/zip_file_replace.o
COMPONENT_OBJS += libzip/lib/zip_file_set_comment.o
COMPONENT_OBJS += libzip/lib/zip_file_set_encryption.o
COMPONENT_OBJS += libzip/lib/zip_file_set_external_attributes.o
COMPONENT_OBJS += libzip/lib/zip_file_set_mtime.o
COMPONENT_OBJS += libzip/lib/zip_file_strerror.o
COMPONENT_OBJS += libzip/lib/zip_filerange_crc.o
COMPONENT_OBJS += libzip/lib/zip_fopen.o
COMPONENT_OBJS += libzip/lib/zip_fopen_encrypted.o
COMPONENT_OBJS += libzip/lib/zip_fopen_index.o
COMPONENT_OBJS += libzip/lib/zip_fopen_index_encrypted.o
COMPONENT_OBJS += libzip/lib/zip_fread.o
COMPONENT_OBJS += libzip/lib/zip_fseek.o
COMPONENT_OBJS += libzip/lib/zip_ftell.o
COMPONENT_OBJS += libzip/lib/zip_get_archive_comment.o
COMPONENT_OBJS += libzip/lib/zip_get_archive_flag.o
COMPONENT_OBJS += libzip/lib/zip_get_encryption_implementation.o
COMPONENT_OBJS += libzip/lib/zip_get_file_comment.o
COMPONENT_OBJS += libzip/lib/zip_get_name.o
COMPONENT_OBJS += libzip/lib/zip_get_num_entries.o
COMPONENT_OBJS += libzip/lib/zip_get_num_files.o
COMPONENT_OBJS += libzip/lib/zip_hash.o
COMPONENT_OBJS += libzip/lib/zip_io_util.o
COMPONENT_OBJS += libzip/lib/zip_libzip_version.o
COMPONENT_OBJS += libzip/lib/zip_memdup.o
COMPONENT_OBJS += libzip/lib/zip_name_locate.o
COMPONENT_OBJS += libzip/lib/zip_new.o
COMPONENT_OBJS += libzip/lib/zip_open.o
COMPONENT_OBJS += libzip/lib/zip_progress.o
COMPONENT_OBJS += libzip/lib/zip_rename.o
COMPONENT_OBJS += libzip/lib/zip_replace.o
COMPONENT_OBJS += libzip/lib/zip_set_archive_comment.o
COMPONENT_OBJS += libzip/lib/zip_set_archive_flag.o
COMPONENT_OBJS += libzip/lib/zip_set_default_password.o
COMPONENT_OBJS += libzip/lib/zip_set_file_comment.o
COMPONENT_OBJS += libzip/lib/zip_set_file_compression.o
COMPONENT_OBJS += libzip/lib/zip_set_name.o
COMPONENT_OBJS += libzip/lib/zip_source_begin_write.o
COMPONENT_OBJS += libzip/lib/zip_source_begin_write_cloning.o
COMPONENT_OBJS += libzip/lib/zip_source_buffer.o
COMPONENT_OBJS += libzip/lib/zip_source_call.o
COMPONENT_OBJS += libzip/lib/zip_source_close.o
COMPONENT_OBJS += libzip/lib/zip_source_commit_write.o
COMPONENT_OBJS += libzip/lib/zip_source_compress.o
COMPONENT_OBJS += libzip/lib/zip_source_crc.o
COMPONENT_OBJS += libzip/lib/zip_source_error.o
COMPONENT_OBJS += libzip/lib/zip_source_file.o
COMPONENT_OBJS += libzip/lib/zip_source_filep.o
COMPONENT_OBJS += libzip/lib/zip_source_free.o
COMPONENT_OBJS += libzip/lib/zip_source_function.o
COMPONENT_OBJS += libzip/lib/zip_source_get_compression_flags.o
COMPONENT_OBJS += libzip/lib/zip_source_is_deleted.o
COMPONENT_OBJS += libzip/lib/zip_source_layered.o
COMPONENT_OBJS += libzip/lib/zip_source_open.o
COMPONENT_OBJS += libzip/lib/zip_source_pkware.o
COMPONENT_OBJS += libzip/lib/zip_source_read.o
COMPONENT_OBJS += libzip/lib/zip_source_remove.o
COMPONENT_OBJS += libzip/lib/zip_source_rollback_write.o
COMPONENT_OBJS += libzip/lib/zip_source_seek.o
COMPONENT_OBJS += libzip/lib/zip_source_seek_write.o
COMPONENT_OBJS += libzip/lib/zip_source_stat.o
COMPONENT_OBJS += libzip/lib/zip_source_supports.o
COMPONENT_OBJS += libzip/lib/zip_source_tell.o
COMPONENT_OBJS += libzip/lib/zip_source_tell_write.o
COMPONENT_OBJS += libzip/lib/zip_source_window.o
COMPONENT_OBJS += libzip/lib/zip_source_winzip_aes_decode.o
COMPONENT_OBJS += libzip/lib/zip_source_winzip_aes_encode.o
COMPONENT_OBJS += libzip/lib/zip_source_write.o
COMPONENT_OBJS += libzip/lib/zip_source_zip.o
COMPONENT_OBJS += libzip/lib/zip_source_zip_new.o
COMPONENT_OBJS += libzip/lib/zip_stat.o
COMPONENT_OBJS += libzip/lib/zip_stat_index.o
COMPONENT_OBJS += libzip/lib/zip_stat_init.o
COMPONENT_OBJS += libzip/lib/zip_strerror.o
COMPONENT_OBJS += libzip/lib/zip_string.o
COMPONENT_OBJS += libzip/lib/zip_unchange.o
COMPONENT_OBJS += libzip/lib/zip_unchange_all.o
COMPONENT_OBJS += libzip/lib/zip_unchange_archive.o
COMPONENT_OBJS += libzip/lib/zip_unchange_data.o
COMPONENT_OBJS += libzip/lib/zip_utf-8.o
COMPONENT_OBJS += libzip/lib/zip_winzip_aes.o
