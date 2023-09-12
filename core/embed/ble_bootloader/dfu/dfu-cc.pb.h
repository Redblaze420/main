/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5 */

#ifndef PB_DFU_DFU_CC_PB_H_INCLUDED
#define PB_DFU_DFU_CC_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _dfu_FwType { 
    dfu_FwType_APPLICATION = 0, 
    dfu_FwType_SOFTDEVICE = 1, 
    dfu_FwType_BOOTLOADER = 2, 
    dfu_FwType_SOFTDEVICE_BOOTLOADER = 3, 
    dfu_FwType_EXTERNAL_APPLICATION = 4 
} dfu_FwType;

typedef enum _dfu_HashType { 
    dfu_HashType_NO_HASH = 0, 
    dfu_HashType_CRC = 1, 
    dfu_HashType_SHA128 = 2, 
    dfu_HashType_SHA256 = 3, 
    dfu_HashType_SHA512 = 4 
} dfu_HashType;

typedef enum _dfu_OpCode { 
    dfu_OpCode_INIT = 1 
} dfu_OpCode;

/* Struct definitions */
typedef PB_BYTES_ARRAY_T(64) dfu_BootValidation_bytes_t;
typedef struct _dfu_BootValidation { 
    uint32_t sigmask; 
    dfu_BootValidation_bytes_t bytes; 
} dfu_BootValidation;

typedef PB_BYTES_ARRAY_T(32) dfu_Hash_hash_t;
typedef struct _dfu_Hash { 
    dfu_HashType hash_type; 
    dfu_Hash_hash_t hash; 
} dfu_Hash;

/* Commands data */
typedef struct _dfu_InitCommand { 
    bool has_fw_version;
    uint32_t fw_version; 
    bool has_hw_version;
    uint32_t hw_version; 
    pb_size_t sd_req_count;
    uint32_t sd_req[16]; 
    bool has_type;
    dfu_FwType type; 
    bool has_sd_size;
    uint32_t sd_size; 
    bool has_bl_size;
    uint32_t bl_size; 
    bool has_app_size;
    uint32_t app_size; 
    bool has_hash;
    dfu_Hash hash; 
    bool has_is_debug;
    bool is_debug; 
    pb_size_t boot_validation_count;
    dfu_BootValidation boot_validation[3]; 
} dfu_InitCommand;

/* Command type */
typedef struct _dfu_Command { 
    bool has_op_code;
    dfu_OpCode op_code; 
    bool has_init;
    dfu_InitCommand init; 
} dfu_Command;

typedef PB_BYTES_ARRAY_T(64) dfu_SignedCommand_signature_t;
typedef struct _dfu_SignedCommand { 
    dfu_Command command; 
    uint32_t sigmask; 
    dfu_SignedCommand_signature_t signature; 
} dfu_SignedCommand;

/* Parent packet type */
typedef struct _dfu_Packet { 
    bool has_command;
    dfu_Command command; 
    bool has_signed_command;
    dfu_SignedCommand signed_command; 
} dfu_Packet;


/* Helper constants for enums */
#define _dfu_FwType_MIN dfu_FwType_APPLICATION
#define _dfu_FwType_MAX dfu_FwType_EXTERNAL_APPLICATION
#define _dfu_FwType_ARRAYSIZE ((dfu_FwType)(dfu_FwType_EXTERNAL_APPLICATION+1))

#define _dfu_HashType_MIN dfu_HashType_NO_HASH
#define _dfu_HashType_MAX dfu_HashType_SHA512
#define _dfu_HashType_ARRAYSIZE ((dfu_HashType)(dfu_HashType_SHA512+1))

#define _dfu_OpCode_MIN dfu_OpCode_INIT
#define _dfu_OpCode_MAX dfu_OpCode_INIT
#define _dfu_OpCode_ARRAYSIZE ((dfu_OpCode)(dfu_OpCode_INIT+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define dfu_Hash_init_default                    {_dfu_HashType_MIN, {0, {0}}}
#define dfu_BootValidation_init_default          {0, {0, {0}}}
#define dfu_InitCommand_init_default             {false, 0, false, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, false, _dfu_FwType_MIN, false, 0, false, 0, false, 0, false, dfu_Hash_init_default, false, false, 0, {dfu_BootValidation_init_default, dfu_BootValidation_init_default, dfu_BootValidation_init_default}}
#define dfu_Command_init_default                 {false, _dfu_OpCode_MIN, false, dfu_InitCommand_init_default}
#define dfu_SignedCommand_init_default           {dfu_Command_init_default, 0, {0, {0}}}
#define dfu_Packet_init_default                  {false, dfu_Command_init_default, false, dfu_SignedCommand_init_default}
#define dfu_Hash_init_zero                       {_dfu_HashType_MIN, {0, {0}}}
#define dfu_BootValidation_init_zero             {0, {0, {0}}}
#define dfu_InitCommand_init_zero                {false, 0, false, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, false, _dfu_FwType_MIN, false, 0, false, 0, false, 0, false, dfu_Hash_init_zero, false, 0, 0, {dfu_BootValidation_init_zero, dfu_BootValidation_init_zero, dfu_BootValidation_init_zero}}
#define dfu_Command_init_zero                    {false, _dfu_OpCode_MIN, false, dfu_InitCommand_init_zero}
#define dfu_SignedCommand_init_zero              {dfu_Command_init_zero, 0, {0, {0}}}
#define dfu_Packet_init_zero                     {false, dfu_Command_init_zero, false, dfu_SignedCommand_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define dfu_BootValidation_sigmask_tag           1
#define dfu_BootValidation_bytes_tag             2
#define dfu_Hash_hash_type_tag                   1
#define dfu_Hash_hash_tag                        2
#define dfu_InitCommand_fw_version_tag           1
#define dfu_InitCommand_hw_version_tag           2
#define dfu_InitCommand_sd_req_tag               3
#define dfu_InitCommand_type_tag                 4
#define dfu_InitCommand_sd_size_tag              5
#define dfu_InitCommand_bl_size_tag              6
#define dfu_InitCommand_app_size_tag             7
#define dfu_InitCommand_hash_tag                 8
#define dfu_InitCommand_is_debug_tag             9
#define dfu_InitCommand_boot_validation_tag      10
#define dfu_Command_op_code_tag                  1
#define dfu_Command_init_tag                     2
#define dfu_SignedCommand_command_tag            1
#define dfu_SignedCommand_sigmask_tag            2
#define dfu_SignedCommand_signature_tag          3
#define dfu_Packet_command_tag                   1
#define dfu_Packet_signed_command_tag            2

/* Struct field encoding specification for nanopb */
#define dfu_Hash_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UENUM,    hash_type,         1) \
X(a, STATIC,   REQUIRED, BYTES,    hash,              2)
#define dfu_Hash_CALLBACK NULL
#define dfu_Hash_DEFAULT NULL

#define dfu_BootValidation_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   sigmask,           1) \
X(a, STATIC,   REQUIRED, BYTES,    bytes,             2)
#define dfu_BootValidation_CALLBACK NULL
#define dfu_BootValidation_DEFAULT NULL

#define dfu_InitCommand_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UINT32,   fw_version,        1) \
X(a, STATIC,   OPTIONAL, UINT32,   hw_version,        2) \
X(a, STATIC,   REPEATED, UINT32,   sd_req,            3) \
X(a, STATIC,   OPTIONAL, UENUM,    type,              4) \
X(a, STATIC,   OPTIONAL, UINT32,   sd_size,           5) \
X(a, STATIC,   OPTIONAL, UINT32,   bl_size,           6) \
X(a, STATIC,   OPTIONAL, UINT32,   app_size,          7) \
X(a, STATIC,   OPTIONAL, MESSAGE,  hash,              8) \
X(a, STATIC,   OPTIONAL, BOOL,     is_debug,          9) \
X(a, STATIC,   REPEATED, MESSAGE,  boot_validation,  10)
#define dfu_InitCommand_CALLBACK NULL
#define dfu_InitCommand_DEFAULT (const pb_byte_t*)"\x48\x00\x00"
#define dfu_InitCommand_hash_MSGTYPE dfu_Hash
#define dfu_InitCommand_boot_validation_MSGTYPE dfu_BootValidation

#define dfu_Command_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UENUM,    op_code,           1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  init,              2)
#define dfu_Command_CALLBACK NULL
#define dfu_Command_DEFAULT (const pb_byte_t*)"\x08\x01\x00"
#define dfu_Command_init_MSGTYPE dfu_InitCommand

#define dfu_SignedCommand_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  command,           1) \
X(a, STATIC,   REQUIRED, UINT32,   sigmask,           2) \
X(a, STATIC,   REQUIRED, BYTES,    signature,         3)
#define dfu_SignedCommand_CALLBACK NULL
#define dfu_SignedCommand_DEFAULT NULL
#define dfu_SignedCommand_command_MSGTYPE dfu_Command

#define dfu_Packet_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  command,           1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  signed_command,    2)
#define dfu_Packet_CALLBACK NULL
#define dfu_Packet_DEFAULT NULL
#define dfu_Packet_command_MSGTYPE dfu_Command
#define dfu_Packet_signed_command_MSGTYPE dfu_SignedCommand

extern const pb_msgdesc_t dfu_Hash_msg;
extern const pb_msgdesc_t dfu_BootValidation_msg;
extern const pb_msgdesc_t dfu_InitCommand_msg;
extern const pb_msgdesc_t dfu_Command_msg;
extern const pb_msgdesc_t dfu_SignedCommand_msg;
extern const pb_msgdesc_t dfu_Packet_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define dfu_Hash_fields &dfu_Hash_msg
#define dfu_BootValidation_fields &dfu_BootValidation_msg
#define dfu_InitCommand_fields &dfu_InitCommand_msg
#define dfu_Command_fields &dfu_Command_msg
#define dfu_SignedCommand_fields &dfu_SignedCommand_msg
#define dfu_Packet_fields &dfu_Packet_msg

/* Maximum encoded size of messages (where known) */
#define dfu_BootValidation_size                  72
#define dfu_Command_size                         395
#define dfu_Hash_size                            36
#define dfu_InitCommand_size                     390
#define dfu_Packet_size                          871
#define dfu_SignedCommand_size                   470

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
