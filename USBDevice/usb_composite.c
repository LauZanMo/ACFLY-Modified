#include "usb_composite.h"
#include "bsp_driver_sd.h"
uint8_t Lock_SD( double TIMEOUT );
void UnLock_SD(void);
bool Get_SD_Init_Complete(void);
void Set_SD_Init_Complete(void);
void Clear_SD_Init_Complete(void);
#include "cmsis_os.h"
#include "message_buffer.h"
#include "Basic.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

#define usbd_c 1
#define tusb_c 1
#define dcd_synopsys_c 1
#define msc_device_c 1
#define msc_disk_c 1
#define cdc_device_c  1
#define usb_descriptors_c 1
#define usb_contronl_c 1

extern SemaphoreHandle_t USB_Semphr;

#if msc_disk_c
extern SD_HandleTypeDef hsd1;

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
  (void) lun;
  const char vid[] = "ACFLy";
  const char pid[] = "Mass Storage";
  const char rev[] = "2.0";
  memcpy(vendor_id  , vid, strlen(vid));
  memcpy(product_id , pid, strlen(pid));
  memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
  static int8_t prev_status = -1;
	bool ret = false;
	if( Get_SD_Init_Complete() )
	{				
		prev_status = 0;
	}	
	if(Lock_SD(-1))
	{
		if (BSP_SD_IsDetected() != SD_NOT_PRESENT)
		{
			if (prev_status < 0)
			{				
				if(!Get_SD_Init_Complete())
				{
					if(BSP_SD_Init()==MSD_OK)
					{
						SD_Driver.disk_initialize(0);	
						Set_SD_Init_Complete();
						prev_status = 0;
					}
				}					
			}
			if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
			{
				ret = true;
			}
		}
		UnLock_SD();
	}
  else if (prev_status == 0)
  {
    prev_status = -1;
  }	
  return ret; 
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
  (void) lun;
  HAL_SD_CardInfoTypeDef info;
	if(Lock_SD(-1))
	{
		if (BSP_SD_IsDetected() != SD_NOT_PRESENT)
		{
			BSP_SD_GetCardInfo(&info);
			*block_count = info.LogBlockNbr - 1;
			*block_size = info.LogBlockSize;
		}			
		UnLock_SD();
	}
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
  (void) lun;
  (void) power_condition;

  if ( load_eject )
  {
    if (start)
    {
      // load disk storage
    }else
    {
      // unload disk storage
    }
  }
  return true;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
	uint8_t ret =1;
	uint32_t timer;
	uint32_t current_blk_addr = lba+offset;
	uint16_t blk_len = bufsize/BLOCKSIZE;
	static uint32_t fifo_addr=0;
	static uint8_t fifo_blk_len=0;
	
	if(BSP_SD_IsDetected() != SD_NOT_PRESENT)
	{	
		if( SD_Driver.disk_read( 0, buffer, current_blk_addr, blk_len ) == RES_OK )
			return blk_len*BLOCKSIZE;
		else
			return 0;
	}
  return 0;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
  int8_t ret = 1;
	uint32_t timer;
	uint32_t blk_addr = lba+offset;
	uint16_t blk_len = bufsize/BLOCKSIZE;
	if(BSP_SD_IsDetected() != SD_NOT_PRESENT)
	{	
		if( SD_Driver.disk_write( 0, buffer, blk_addr, blk_len ) == RES_OK )
			return blk_len*BLOCKSIZE;
		else
			return 0;
	}
  return 0;
}
// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t tud_msc_scsi_cb (uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  // read10 & write10 has their own callback and MUST not be handled here
  void const* response = NULL;
  uint16_t resplen = 0;

  // most scsi handled is input
  bool in_xfer = true;

  switch (scsi_cmd[0])
  {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      // Host is about to read/write etc ... better not to disconnect disk
      resplen = 0;
    break;

    default:
      // Set Sense = Invalid Command Operation
      tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

      // negative means error -> tinyusb could stall and/or response with failed status
      resplen = -1;
    break;
  }

  // return resplen must not larger than bufsize
  if ( resplen > bufsize ) resplen = bufsize;

  if ( response && (resplen > 0) )
  {
    if(in_xfer)
    {
      memcpy(buffer, response, resplen);
    }else
    {
      // SCSI output
    }
  }

  return resplen;
}
#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if cdc_device_c 

typedef struct
{
  uint8_t itf_num;
  uint8_t ep_notif;
  uint8_t ep_in;
  uint8_t ep_out;

  // Bit 0:  DTR (Data Terminal Ready), Bit 1: RTS (Request to Send)
  uint8_t line_state;

  /*------------- From this point, data is not cleared by bus reset -------------*/
  char  wanted_char;
  cdc_line_coding_t line_coding;

  // Endpoint Transfer buffer
  CFG_TUSB_MEM_ALIGN uint8_t epout_buf[CFG_TUD_CDC_EP_BUFSIZE];
  CFG_TUSB_MEM_ALIGN uint8_t epin_buf[CFG_TUD_CDC_EP_BUFSIZE];

}cdcd_interface_t;

#define ITF_MEM_RESET_SIZE   offsetof(cdcd_interface_t, wanted_char)

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
CFG_TUSB_MEM_SECTION static cdcd_interface_t _cdcd_itf[CFG_TUD_CDC];

static void _prep_out_transaction (uint8_t itf)
{
  cdcd_interface_t* p_cdc = &_cdcd_itf[itf];
//  // skip if previous transfer not complete
//  if ( usbd_edpt_busy(TUD_OPT_RHPORT, p_cdc->ep_out) ) return;
	 // claim endpoint
	TU_VERIFY( usbd_edpt_claim(CFG_TUD_CDC, p_cdc->ep_out),);
	
	usbd_edpt_xfer(TUD_OPT_RHPORT, p_cdc->ep_out, p_cdc->epout_buf, sizeof(p_cdc->epout_buf));	
}

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+
bool tud_cdc_n_connected(uint8_t itf)
{
  // DTR (bit 0) active  is considered as connected
  return tud_ready() && tu_bit_test(_cdcd_itf[itf].line_state, 0);
}

uint8_t tud_cdc_n_get_line_state (uint8_t itf)
{
  return _cdcd_itf[itf].line_state;
}

void tud_cdc_n_get_line_coding (uint8_t itf, cdc_line_coding_t* coding)
{
  (*coding) = _cdcd_itf[itf].line_coding;
}

void tud_cdc_n_set_wanted_char (uint8_t itf, char wanted)
{
  _cdcd_itf[itf].wanted_char = wanted;
}



extern StreamBufferHandle_t usb_tx_streambufferhandle_t;
uint32_t cdc_write(uint8_t itf)
{
	uint16_t count=0;	
  cdcd_interface_t* p_cdc = &_cdcd_itf[itf];

	if(xStreamBufferBytesAvailable(usb_tx_streambufferhandle_t)<=0)
		return 0;
  // claim the endpoint first
  TU_VERIFY( usbd_edpt_claim(CFG_TUD_CDC, p_cdc->ep_in), 0);	
	count = xStreamBufferReceive( usb_tx_streambufferhandle_t,p_cdc->epin_buf, sizeof(p_cdc->epin_buf),0);	
	if( count )
	{
		xSemaphoreTakeRecursive( USB_Semphr, portMAX_DELAY );
		if(!usbd_edpt_xfer(TUD_OPT_RHPORT, p_cdc->ep_in, p_cdc->epin_buf, count)){
			usbd_edpt_release(CFG_TUD_CDC,p_cdc->ep_in);
			xSemaphoreGiveRecursive( USB_Semphr );
			return 0;	
		}
		xSemaphoreGiveRecursive( USB_Semphr );
	}else
	{
    usbd_edpt_release(CFG_TUD_CDC, p_cdc->ep_in);
    return 0;	
	}
  return count;
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void cdcd_init(void)
{
  tu_memclr(_cdcd_itf, sizeof(_cdcd_itf));

  for(uint8_t i=0; i<CFG_TUD_CDC; i++)
  {
    cdcd_interface_t* p_cdc = &_cdcd_itf[i];

    p_cdc->wanted_char = -1;

    // default line coding is : stop bit = 1, parity = none, data bits = 8
    p_cdc->line_coding.bit_rate = 115200;
    p_cdc->line_coding.stop_bits = 0;
    p_cdc->line_coding.parity    = 0;
    p_cdc->line_coding.data_bits = 8;
  }
}

void cdcd_reset(uint8_t rhport)
{
  (void) rhport;

  for(uint8_t i=0; i<CFG_TUD_CDC; i++)
  {
    tu_memclr(&_cdcd_itf[i], ITF_MEM_RESET_SIZE);
  }
}

uint16_t cdcd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len)
{
  // Only support ACM subclass
  TU_VERIFY ( TUSB_CLASS_CDC                           == itf_desc->bInterfaceClass &&
              CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL == itf_desc->bInterfaceSubClass, 0);

  // Note: 0xFF can be used with RNDIS
  TU_VERIFY(tu_within(CDC_COMM_PROTOCOL_NONE, itf_desc->bInterfaceProtocol, CDC_COMM_PROTOCOL_ATCOMMAND_CDMA), 0);

  // Find available interface
  cdcd_interface_t * p_cdc = NULL;
  uint8_t cdc_id;
  for(cdc_id=0; cdc_id<CFG_TUD_CDC; cdc_id++)
  {
    if ( _cdcd_itf[cdc_id].ep_in == 0 )
    {
      p_cdc = &_cdcd_itf[cdc_id];
      break;
    }
  }
  TU_ASSERT(p_cdc, 0);

  //------------- Control Interface -------------//
  p_cdc->itf_num = itf_desc->bInterfaceNumber;

  uint16_t drv_len = sizeof(tusb_desc_interface_t);
  uint8_t const * p_desc = tu_desc_next( itf_desc );

  // Communication Functional Descriptors
  while ( TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
  {
    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  if ( TUSB_DESC_ENDPOINT == tu_desc_type(p_desc) )
  {
    // notification endpoint if any
    TU_ASSERT( usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *) p_desc), 0 );

    p_cdc->ep_notif = ((tusb_desc_endpoint_t const *) p_desc)->bEndpointAddress;

    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  //------------- Data Interface (if any) -------------//
  if ( (TUSB_DESC_INTERFACE == tu_desc_type(p_desc)) &&
       (TUSB_CLASS_CDC_DATA == ((tusb_desc_interface_t const *) p_desc)->bInterfaceClass) )
  {
    // next to endpoint descriptor
    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);

    // Open endpoint pair
    TU_ASSERT( usbd_open_edpt_pair(rhport, p_desc, 2, TUSB_XFER_BULK, &p_cdc->ep_out, &p_cdc->ep_in), 0 );

    drv_len += 2*sizeof(tusb_desc_endpoint_t);
  }

  // Prepare for incoming data
  _prep_out_transaction(cdc_id);

  return drv_len;
}

// Invoked when class request DATA stage is finished.
// return false to stall control endpoint (e.g Host send non-sense DATA)
bool cdcd_control_complete(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;

  //------------- Class Specific Request -------------//
  TU_VERIFY (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);

  uint8_t itf = 0;
  cdcd_interface_t* p_cdc = _cdcd_itf;

  // Identify which interface to use
  for ( ; ; itf++, p_cdc++)
  {
    if (itf >= TU_ARRAY_SIZE(_cdcd_itf)) return false;

    if ( p_cdc->itf_num == request->wIndex ) break;
  }

  // Invoke callback
  if ( CDC_REQUEST_SET_LINE_CODING == request->bRequest )
  {
    if ( tud_cdc_line_coding_cb ) tud_cdc_line_coding_cb(itf, &p_cdc->line_coding);
  }

  return true;
}

// Handle class control request
// return false to stall control endpoint (e.g unsupported request)
bool cdcd_control_request(uint8_t rhport, tusb_control_request_t * request)
{
  // Handle class request only
  TU_VERIFY(request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);

  uint8_t itf = 0;
  cdcd_interface_t* p_cdc = _cdcd_itf;

  // Identify which interface to use
  for ( ; ; itf++, p_cdc++)
  {
    if (itf >= TU_ARRAY_SIZE(_cdcd_itf)) return false;

    if ( p_cdc->itf_num == request->wIndex ) break;
  }

  switch ( request->bRequest )
  {
    case CDC_REQUEST_SET_LINE_CODING:
      TU_LOG2("  Set Line Coding\r\n");
      tud_control_xfer(rhport, request, &p_cdc->line_coding, sizeof(cdc_line_coding_t));
    break;

    case CDC_REQUEST_GET_LINE_CODING:
      TU_LOG2("  Get Line Coding\r\n");
      tud_control_xfer(rhport, request, &p_cdc->line_coding, sizeof(cdc_line_coding_t));
    break;

    case CDC_REQUEST_SET_CONTROL_LINE_STATE:
    {
      // CDC PSTN v1.2 section 6.3.12
      // Bit 0: Indicates if DTE is present or not.
      //        This signal corresponds to V.24 signal 108/2 and RS-232 signal DTR (Data Terminal Ready)
      // Bit 1: Carrier control for half-duplex modems.
      //        This signal corresponds to V.24 signal 105 and RS-232 signal RTS (Request to Send)
      request->wValue|=(0x03);
			bool  dtr = tu_bit_test(request->wValue, 0);
      bool  rts = tu_bit_test(request->wValue, 1);

      
      p_cdc->line_state = (uint8_t) request->wValue;
      
      TU_LOG2("  Set Control Line State: DTR = %d, RTS = %d\r\n", dtr, rts);

      tud_control_status(rhport, request);

      // Invoke callback
      if ( tud_cdc_line_state_cb ) tud_cdc_line_state_cb(itf, dtr, rts);
    }
    break;

    default: return false; // stall unsupported request
  }

  return true;
}


bool cdcd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) rhport;
  (void) result;

  uint8_t itf;
  cdcd_interface_t* p_cdc;

  // Identify which interface to use
  for (itf = 0; itf < CFG_TUD_CDC; itf++)
  {
    p_cdc = &_cdcd_itf[itf];
    if ( ( ep_addr == p_cdc->ep_out ) || ( ep_addr == p_cdc->ep_in ) ) break;
  }
  TU_ASSERT(itf < CFG_TUD_CDC);

  // Received new data
  if( ep_addr == p_cdc->ep_out )
  {
    // invoke receive callback (if there is still data)		
		tud_cdc_rx_cb(itf,&p_cdc->epout_buf,xferred_bytes);		
    // prepare for OUT transaction
    _prep_out_transaction(itf);
  }

  // Data sent to host, we continue to fetch from tx fifo to send.
  // Note: This will cause incorrect baudrate set in line coding.
  //       Though maybe the baudrate is not really important !!!
  if ( ep_addr == p_cdc->ep_in )
  {
    if ( 0 == cdc_write(itf) )
    {
      // There is no data left, a ZLP should be sent if
      // xferred_bytes is multiple of EP size and not zero
			if (xferred_bytes && (0 == (xferred_bytes % CFG_TUD_CDC_EP_BUFSIZE)) )
			{
				TU_VERIFY( usbd_edpt_claim(CFG_TUD_CDC, p_cdc->ep_in), 0);	
				usbd_edpt_xfer(TUD_OPT_RHPORT, p_cdc->ep_in, NULL, 0);
			}
    }
  }
  // nothing to do with notif endpoint for now
  return true;
}

#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if msc_device_c
enum
{
  MSC_STAGE_CMD  = 0,
  MSC_STAGE_DATA,
  MSC_STAGE_STATUS,
  MSC_STAGE_STATUS_SENT
};

typedef struct
{
  // TODO optimize alignment
  CFG_TUSB_MEM_ALIGN msc_cbw_t cbw;
  CFG_TUSB_MEM_ALIGN msc_csw_t csw;

  uint8_t  itf_num;
  uint8_t  ep_in;
  uint8_t  ep_out;

  // Bulk Only Transfer (BOT) Protocol
  uint8_t  stage;
  uint32_t total_len;
  uint32_t xferred_len; // numbered of bytes transferred so far in the Data Stage

  // Sense Response Data
  uint8_t sense_key;
  uint8_t add_sense_code;
  uint8_t add_sense_qualifier;
}mscd_interface_t;

CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static mscd_interface_t _mscd_itf;
CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t _mscd_buf[CFG_TUD_MSC_EP_BUFSIZE];

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static bool proc_read10_cmd(uint8_t rhport, mscd_interface_t* p_msc);
static bool proc_write10_cmd(uint8_t rhport, mscd_interface_t* p_msc);

static inline uint32_t rdwr10_get_lba(uint8_t const command[])
{
  // read10 & write10 has the same format
  scsi_write10_t* p_rdwr10 = (scsi_write10_t*) command;

  // copy first to prevent mis-aligned access
  uint32_t lba;
  // use offsetof to avoid pointer to the odd/misaligned address
  memcpy(&lba, (uint8_t*) p_rdwr10 + offsetof(scsi_write10_t, lba), 4);

  // lba is in Big Endian format
  return tu_ntohl(lba);
}

static inline uint16_t rdwr10_get_blockcount(uint8_t const command[])
{
  // read10 & write10 has the same format
  scsi_write10_t* p_rdwr10 = (scsi_write10_t*) command;

  // copy first to prevent mis-aligned access
  uint16_t block_count;
  // use offsetof to avoid pointer to the odd/misaligned address
  memcpy(&block_count, (uint8_t*) p_rdwr10 + offsetof(scsi_write10_t, block_count), 2);

  return tu_ntohs(block_count);
}

//--------------------------------------------------------------------+
// Debug
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= 2

static tu_lookup_entry_t const _msc_scsi_cmd_lookup[] =
{
  { .key = SCSI_CMD_TEST_UNIT_READY              , .data = "Test Unit Ready" },
  { .key = SCSI_CMD_INQUIRY                      , .data = "Inquiry" },
  { .key = SCSI_CMD_MODE_SELECT_6                , .data = "Mode_Select 6" },
  { .key = SCSI_CMD_MODE_SENSE_6                 , .data = "Mode_Sense 6" },
  { .key = SCSI_CMD_START_STOP_UNIT              , .data = "Start Stop Unit" },
  { .key = SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL , .data = "Prevent Allow Medium Removal" },
  { .key = SCSI_CMD_READ_CAPACITY_10             , .data = "Read Capacity10" },
  { .key = SCSI_CMD_REQUEST_SENSE                , .data = "Request Sense" },
  { .key = SCSI_CMD_READ_FORMAT_CAPACITY         , .data = "Read Format Capacity" },
  { .key = SCSI_CMD_READ_10                      , .data = "Read10" },
  { .key = SCSI_CMD_WRITE_10                     , .data = "Write10" }
};

static tu_lookup_table_t const _msc_scsi_cmd_table =
{
  .count = TU_ARRAY_SIZE(_msc_scsi_cmd_lookup),
  .items = _msc_scsi_cmd_lookup
};

#endif

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+
bool tud_msc_set_sense(uint8_t lun, uint8_t sense_key, uint8_t add_sense_code, uint8_t add_sense_qualifier)
{
  (void) lun;

  _mscd_itf.sense_key           = sense_key;
  _mscd_itf.add_sense_code      = add_sense_code;
  _mscd_itf.add_sense_qualifier = add_sense_qualifier;

  return true;
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void mscd_init(void)
{
  tu_memclr(&_mscd_itf, sizeof(mscd_interface_t));
}

void mscd_reset(uint8_t rhport)
{
  (void) rhport;
  tu_memclr(&_mscd_itf, sizeof(mscd_interface_t));
}

uint16_t mscd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len)
{
  // only support SCSI's BOT protocol
  TU_VERIFY(TUSB_CLASS_MSC    == itf_desc->bInterfaceClass &&
            MSC_SUBCLASS_SCSI == itf_desc->bInterfaceSubClass &&
            MSC_PROTOCOL_BOT  == itf_desc->bInterfaceProtocol, 0);

  // msc driver length is fixed
  uint16_t const drv_len = sizeof(tusb_desc_interface_t) + 2*sizeof(tusb_desc_endpoint_t);

  // Max length mus be at least 1 interface + 2 endpoints
  TU_ASSERT(max_len >= drv_len, 0);

  mscd_interface_t * p_msc = &_mscd_itf;
  p_msc->itf_num = itf_desc->bInterfaceNumber;

  // Open endpoint pair
  TU_ASSERT( usbd_open_edpt_pair(rhport, tu_desc_next(itf_desc), 2, TUSB_XFER_BULK, &p_msc->ep_out, &p_msc->ep_in), 0 );

  // Prepare for Command Block Wrapper
  if ( !usbd_edpt_xfer(rhport, p_msc->ep_out, (uint8_t*) &p_msc->cbw, sizeof(msc_cbw_t)) )
  {
    TU_LOG1_FAILED();
    TU_BREAKPOINT();
  }

  return drv_len;
}

// Handle class control request
// return false to stall control endpoint (e.g unsupported request)
bool mscd_control_request(uint8_t rhport, tusb_control_request_t * p_request)
{
  // Handle class request only
  TU_VERIFY(p_request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);

  switch ( p_request->bRequest )
  {
    case MSC_REQ_RESET:
      // TODO: Actually reset interface.
      tud_control_status(rhport, p_request);
    break;

    case MSC_REQ_GET_MAX_LUN:
    {
      uint8_t maxlun = 1;
      if (tud_msc_get_maxlun_cb) maxlun = tud_msc_get_maxlun_cb();
      TU_VERIFY(maxlun);

      // MAX LUN is minus 1 by specs
      maxlun--;

      tud_control_xfer(rhport, p_request, &maxlun, 1);
    }
    break;

    default: return false; // stall unsupported request
  }

  return true;
}

// Invoked when class request DATA stage is finished.
// return false to stall control endpoint (e.g Host send non-sense DATA)
bool mscd_control_complete(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;
  (void) request;

  // nothing to do
  return true;
}

// return response's length (copied to buffer). Negative if it is not an built-in command or indicate Failed status (CSW)
// In case of a failed status, sense key must be set for reason of failure
int32_t proc_builtin_scsi(uint8_t lun, uint8_t const scsi_cmd[16], uint8_t* buffer, uint32_t bufsize)
{
  (void) bufsize; // TODO refractor later
  int32_t resplen;

  switch ( scsi_cmd[0] )
  {
    case SCSI_CMD_TEST_UNIT_READY:
      resplen = 0;
      if ( !tud_msc_test_unit_ready_cb(lun) )
      {
        // Failed status response
        resplen = - 1;

        // If sense key is not set by callback, default to Logical Unit Not Ready, Cause Not Reportable
        if ( _mscd_itf.sense_key == 0 ) tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x00);
      }
    break;

    case SCSI_CMD_START_STOP_UNIT:
      resplen = 0;

      if (tud_msc_start_stop_cb)
      {
        scsi_start_stop_unit_t const * start_stop = (scsi_start_stop_unit_t const *) scsi_cmd;
        if ( !tud_msc_start_stop_cb(lun, start_stop->power_condition, start_stop->start, start_stop->load_eject) )
        {
          // Failed status response
          resplen = - 1;

          // If sense key is not set by callback, default to Logical Unit Not Ready, Cause Not Reportable
          if ( _mscd_itf.sense_key == 0 ) tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x00);
        }
      }
    break;

    case SCSI_CMD_READ_CAPACITY_10:
    {
      uint32_t block_count;
      uint32_t block_size;
      uint16_t block_size_u16;

      tud_msc_capacity_cb(lun, &block_count, &block_size_u16);
      block_size = (uint32_t) block_size_u16;

      // Invalid block size/count from callback, possibly unit is not ready
      // stall this request, set sense key to NOT READY
      if (block_count == 0 || block_size == 0)
      {
        resplen = -1;

        // If sense key is not set by callback, default to Logical Unit Not Ready, Cause Not Reportable
        if ( _mscd_itf.sense_key == 0 ) tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x00);
      }else
      {
        scsi_read_capacity10_resp_t read_capa10;

        read_capa10.last_lba = tu_htonl(block_count-1);
        read_capa10.block_size = tu_htonl(block_size);

        resplen = sizeof(read_capa10);
        memcpy(buffer, &read_capa10, resplen);
      }
    }
    break;

    case SCSI_CMD_READ_FORMAT_CAPACITY:
    {
      scsi_read_format_capacity_data_t read_fmt_capa =
      {
          .list_length     = 8,
          .block_num       = 0,
          .descriptor_type = 2, // formatted media
          .block_size_u16  = 0
      };

      uint32_t block_count;
      uint16_t block_size;

      tud_msc_capacity_cb(lun, &block_count, &block_size);

      // Invalid block size/count from callback, possibly unit is not ready
      // stall this request, set sense key to NOT READY
      if (block_count == 0 || block_size == 0)
      {
        resplen = -1;

        // If sense key is not set by callback, default to Logical Unit Not Ready, Cause Not Reportable
        if ( _mscd_itf.sense_key == 0 ) tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x00);
      }else
      {
        read_fmt_capa.block_num = tu_htonl(block_count);
        read_fmt_capa.block_size_u16 = tu_htons(block_size);

        resplen = sizeof(read_fmt_capa);
        memcpy(buffer, &read_fmt_capa, resplen);
      }
    }
    break;

    case SCSI_CMD_INQUIRY:
    {
      scsi_inquiry_resp_t inquiry_rsp =
      {
          .is_removable         = 1,
          .version              = 2,
          .response_data_format = 2,
      };

      // vendor_id, product_id, product_rev is space padded string
      memset(inquiry_rsp.vendor_id  , ' ', sizeof(inquiry_rsp.vendor_id));
      memset(inquiry_rsp.product_id , ' ', sizeof(inquiry_rsp.product_id));
      memset(inquiry_rsp.product_rev, ' ', sizeof(inquiry_rsp.product_rev));

      tud_msc_inquiry_cb(lun, inquiry_rsp.vendor_id, inquiry_rsp.product_id, inquiry_rsp.product_rev);

      resplen = sizeof(inquiry_rsp);
      memcpy(buffer, &inquiry_rsp, resplen);
    }
    break;

    case SCSI_CMD_MODE_SENSE_6:
    {
      scsi_mode_sense6_resp_t mode_resp =
      {
          .data_len = 3,
          .medium_type = 0,
          .write_protected = false,
          .reserved = 0,
          .block_descriptor_len = 0  // no block descriptor are included
      };

      bool writable = true;
      writable = tud_msc_is_writable_cb(lun);
      mode_resp.write_protected = !writable;

      resplen = sizeof(mode_resp);
      memcpy(buffer, &mode_resp, resplen);
    }
    break;

    case SCSI_CMD_REQUEST_SENSE:
    {
      scsi_sense_fixed_resp_t sense_rsp =
      {
          .response_code = 0x70,
          .valid         = 1
      };

      sense_rsp.add_sense_len = sizeof(scsi_sense_fixed_resp_t) - 8;

      sense_rsp.sense_key           = _mscd_itf.sense_key;
      sense_rsp.add_sense_code      = _mscd_itf.add_sense_code;
      sense_rsp.add_sense_qualifier = _mscd_itf.add_sense_qualifier;

      resplen = sizeof(sense_rsp);
      memcpy(buffer, &sense_rsp, resplen);

      // Clear sense data after copy
      tud_msc_set_sense(lun, 0, 0, 0);
    }
    break;

    default: resplen = -1; break;
  }

  return resplen;
}
static bool race = false;
bool mscd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
  mscd_interface_t* p_msc = &_mscd_itf;
  msc_cbw_t const * p_cbw = &p_msc->cbw;
  msc_csw_t       * p_csw = &p_msc->csw;
	
	if(race)
	{
		return false;
	}
		race = true; 
  switch (p_msc->stage)
  {
    case MSC_STAGE_CMD:{
      //------------- new CBW received -------------//
      // Complete IN while waiting for CMD is usually Status of previous SCSI op, ignore it
      if(ep_addr != p_msc->ep_out) return true;

      TU_ASSERT( event == XFER_RESULT_SUCCESS &&
                 xferred_bytes == sizeof(msc_cbw_t) && p_cbw->signature == MSC_CBW_SIGNATURE );

      TU_LOG2("  SCSI Command: %s\r\n", tu_lookup_find(&_msc_scsi_cmd_table, p_cbw->command[0]));
      // TU_LOG2_MEM(p_cbw, xferred_bytes, 2);

      p_csw->signature    = MSC_CSW_SIGNATURE;
      p_csw->tag          = p_cbw->tag;
      p_csw->data_residue = 0;

      /*------------- Parse command and prepare DATA -------------*/
      p_msc->stage = MSC_STAGE_DATA;
      p_msc->total_len = p_cbw->total_bytes;
      p_msc->xferred_len = 0;

      if (SCSI_CMD_READ_10 == p_cbw->command[0])
      {
        proc_read10_cmd(rhport, p_msc);
      }
      else if (SCSI_CMD_WRITE_10 == p_cbw->command[0])
      {
        proc_write10_cmd(rhport, p_msc);
      }
      else
      {
        // For other SCSI commands
        // 1. OUT : queue transfer (invoke app callback after done)
        // 2. IN & Zero: Process if is built-in, else Invoke app callback. Skip DATA if zero length
        if ( (p_cbw->total_bytes > 0 ) && !tu_bit_test(p_cbw->dir, 7) )
        {
          // queue transfer
          TU_ASSERT( usbd_edpt_xfer(rhport, p_msc->ep_out, _mscd_buf, p_msc->total_len) );
        }else
        { 
          int32_t resplen;

          // First process if it is a built-in commands
          resplen = proc_builtin_scsi(p_cbw->lun, p_cbw->command, _mscd_buf, sizeof(_mscd_buf));

          // Not built-in, invoke user callback
          if ( (resplen < 0) && (p_msc->sense_key == 0) )
          {
            resplen = tud_msc_scsi_cb(p_cbw->lun, p_cbw->command, _mscd_buf, p_msc->total_len);
          }

          if ( resplen < 0 )
          {
            p_msc->total_len = 0;
            p_csw->status = MSC_CSW_STATUS_FAILED;
            p_msc->stage = MSC_STAGE_STATUS;

            // failed but senskey is not set: default to Illegal Request
            if ( p_msc->sense_key == 0 ) tud_msc_set_sense(p_cbw->lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

            // Stall bulk In if needed
            if (p_cbw->total_bytes) usbd_edpt_stall(rhport, p_msc->ep_in);
          }
          else
          {
            p_msc->total_len = (uint32_t) resplen;
            p_csw->status = MSC_CSW_STATUS_PASSED;

            if (p_msc->total_len)
            {
              TU_ASSERT( p_cbw->total_bytes >= p_msc->total_len ); // cannot return more than host expect
              TU_ASSERT( usbd_edpt_xfer(rhport, p_msc->ep_in, _mscd_buf, p_msc->total_len) );
            }else
            {
              p_msc->stage = MSC_STAGE_STATUS;
            }
          }
        }
      }
    break;
		}

    case MSC_STAGE_DATA:{
      TU_LOG2("  SCSI Data\r\n");
      //TU_LOG2_MEM(_mscd_buf, xferred_bytes, 2);

      // OUT transfer, invoke callback if needed
      if ( !tu_bit_test(p_cbw->dir, 7) )
      {
        if ( SCSI_CMD_WRITE_10 != p_cbw->command[0] )
        {
          int32_t cb_result = tud_msc_scsi_cb(p_cbw->lun, p_cbw->command, _mscd_buf, p_msc->total_len);

          if ( cb_result < 0 )
          {
            p_csw->status = MSC_CSW_STATUS_FAILED;
            tud_msc_set_sense(p_cbw->lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00); // Sense = Invalid Command Operation
          }else
          {
            p_csw->status = MSC_CSW_STATUS_PASSED;
          }
        }
        else
        {
          uint16_t const block_sz = p_cbw->total_bytes / rdwr10_get_blockcount(p_cbw->command);

          // Adjust lba with transferred bytes
          uint32_t const lba = rdwr10_get_lba(p_cbw->command) + (p_msc->xferred_len / block_sz);

          // Application can consume smaller bytes
          int32_t nbytes = tud_msc_write10_cb(p_cbw->lun, lba, p_msc->xferred_len % block_sz, _mscd_buf, xferred_bytes);

          if ( nbytes < 0 )
          {
            // negative means error -> skip to status phase, status in CSW set to failed
            p_csw->data_residue = p_cbw->total_bytes - p_msc->xferred_len;
            p_csw->status       = MSC_CSW_STATUS_FAILED;
            p_msc->stage        = MSC_STAGE_STATUS;

            tud_msc_set_sense(p_cbw->lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00); // Sense = Invalid Command Operation
            break;
          }else
          {
            // Application consume less than what we got (including zero)
            if ( nbytes < (int32_t) xferred_bytes )
            {
              if ( nbytes > 0 )
              {
                p_msc->xferred_len += nbytes;
                memmove(_mscd_buf, _mscd_buf+nbytes, xferred_bytes-nbytes);
              }

              // simulate an transfer complete with adjusted parameters --> this driver callback will fired again
              dcd_event_xfer_complete(rhport, p_msc->ep_out, xferred_bytes-nbytes, XFER_RESULT_SUCCESS, false);
              return true; // skip the rest
            }
            else
            {
              // Application consume all bytes in our buffer. Nothing to do, process with normal flow
            }
          }
        }
      }

      // Accumulate data so far
      p_msc->xferred_len += xferred_bytes;

      if ( p_msc->xferred_len >= p_msc->total_len )
      {
        // Data Stage is complete
        p_msc->stage = MSC_STAGE_STATUS;
      }
      else
      {
        // READ10 & WRITE10 Can be executed with large bulk of data e.g write 8K bytes (several flash write)
        // We break it into multiple smaller command whose data size is up to CFG_TUD_MSC_EP_BUFSIZE
        if (SCSI_CMD_READ_10 == p_cbw->command[0])
        {
          proc_read10_cmd(rhport, p_msc);
        }
        else if (SCSI_CMD_WRITE_10 == p_cbw->command[0])
        {
          proc_write10_cmd(rhport, p_msc);
        }
				else
        {
          // No other command take more than one transfer yet -> unlikely error
          TU_BREAKPOINT();
        }
      }
    break;
		}

    case MSC_STAGE_STATUS:{
      // processed immediately after this switch, supposedly to be empty
    break;
	  } 
    case MSC_STAGE_STATUS_SENT:{
      // Wait for the Status phase to complete
      if( (ep_addr == p_msc->ep_in) && (xferred_bytes == sizeof(msc_csw_t)) )
      {
        TU_LOG2("  SCSI Status: %u\r\n", p_csw->status);
        // TU_LOG2_MEM(p_csw, xferred_bytes, 2);

        // Move to default CMD stage
        p_msc->stage = MSC_STAGE_CMD;

        // Queue for the next CBW
        TU_ASSERT( usbd_edpt_xfer(rhport, p_msc->ep_out, (uint8_t*) &p_msc->cbw, sizeof(msc_cbw_t)) );
      }
    break;
    }
    default : break;
  }

  if ( p_msc->stage == MSC_STAGE_STATUS )
  {
    // Either endpoints is stalled, need to wait until it is cleared by host
    if ( usbd_edpt_stalled(rhport,  p_msc->ep_in) || usbd_edpt_stalled(rhport,  p_msc->ep_out) )
    {
      // simulate an transfer complete with adjusted parameters --> this driver callback will fired again
      // and response with status phase after halted endpoints are cleared.
      // note: use ep_out to prevent confusing with STATUS complete
      dcd_event_xfer_complete(rhport, p_msc->ep_out, 0, XFER_RESULT_SUCCESS, false);
    }
    else
    {
      // Invoke complete callback if defined
      // Note: There is racing issue with samd51 + qspi flash testing with arduino
      // if complete_cb() is invoked after queuing the status.
      switch(p_cbw->command[0])
      {
        case SCSI_CMD_READ_10:
          if ( tud_msc_read10_complete_cb ) tud_msc_read10_complete_cb(p_cbw->lun);
        break;

        case SCSI_CMD_WRITE_10:
          if ( tud_msc_write10_complete_cb ) tud_msc_write10_complete_cb(p_cbw->lun);
        break;

        default:
          if ( tud_msc_scsi_complete_cb ) tud_msc_scsi_complete_cb(p_cbw->lun, p_cbw->command);
        break;
      }

      // Move to Status Sent stage
      p_msc->stage = MSC_STAGE_STATUS_SENT;

      // Send SCSI Status
      TU_ASSERT(usbd_edpt_xfer(rhport, p_msc->ep_in , (uint8_t*) &p_msc->csw, sizeof(msc_csw_t)));
    }
  }
  race = false;
  return true;
}

/*------------------------------------------------------------------*/
/* SCSI Command Process
 *------------------------------------------------------------------*/
static bool proc_read10_cmd(uint8_t rhport, mscd_interface_t* p_msc)
{
  msc_cbw_t const * p_cbw = &p_msc->cbw;
  msc_csw_t       * p_csw = &p_msc->csw;

  uint16_t const block_cnt = rdwr10_get_blockcount(p_cbw->command);
  TU_VERIFY(block_cnt, 0); // prevent div by zero

  uint16_t const block_sz = p_cbw->total_bytes / block_cnt;
  TU_VERIFY(block_sz, 0); // prevent div by zero

  // Adjust lba with transferred bytes
  uint32_t const lba = rdwr10_get_lba(p_cbw->command) + (p_msc->xferred_len / block_sz);

  // remaining bytes capped at class buffer
  int32_t nbytes = (int32_t) tu_min32(sizeof(_mscd_buf), p_cbw->total_bytes-p_msc->xferred_len);

  // Application can consume smaller bytes
  int32_t readbytes = tud_msc_read10_cb(p_cbw->lun, lba, p_msc->xferred_len % block_sz, _mscd_buf, (uint32_t) nbytes);

  if ( readbytes < 0 )
  {
    // negative means error -> pipe is stalled & status in CSW set to failed
    p_csw->data_residue = p_cbw->total_bytes - p_msc->xferred_len;
    p_csw->status       = MSC_CSW_STATUS_FAILED;

    tud_msc_set_sense(p_cbw->lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00); // Sense = Invalid Command Operation
    usbd_edpt_stall(rhport, p_msc->ep_in);
  }
  else if ( readbytes == 0 )
  {
    // zero means not ready -> simulate an transfer complete so that this driver callback will fired again
    dcd_event_xfer_complete(rhport, p_msc->ep_in, 0, XFER_RESULT_SUCCESS, false);
  }
  else
  {
    TU_VERIFY( usbd_edpt_xfer(rhport, p_msc->ep_in, _mscd_buf, readbytes), 0);
  }
	return true;
}

static bool proc_write10_cmd(uint8_t rhport, mscd_interface_t* p_msc)
{
  msc_cbw_t const * p_cbw = &p_msc->cbw;
  bool writable = true;
  writable = tud_msc_is_writable_cb(p_cbw->lun);
  if (!writable) {
    msc_csw_t* p_csw = &p_msc->csw;
    p_csw->data_residue = p_cbw->total_bytes;
    p_csw->status       = MSC_CSW_STATUS_FAILED;

    tud_msc_set_sense(p_cbw->lun, SCSI_SENSE_DATA_PROTECT, 0x27, 0x00); // Sense = Write protected
    usbd_edpt_stall(rhport, p_msc->ep_out);
    return 0;
  }

  // remaining bytes capped at class buffer
  int32_t nbytes = (int32_t) tu_min32(sizeof(_mscd_buf), p_cbw->total_bytes-p_msc->xferred_len);

  // Write10 callback will be called later when usb transfer complete
  TU_VERIFY( usbd_edpt_xfer(rhport, p_msc->ep_out, _mscd_buf, nbytes), 0);
	return true;
}


#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if dcd_synopsys_c
#define USE_SOF     0

#include "stm32h7xx.h"
#define EP_MAX_FS       9
#define EP_FIFO_SIZE_FS 4096
#define EP_MAX_HS       9
#define EP_FIFO_SIZE_HS 4096


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

// On STM32 we associate Port0 to OTG_FS, and Port1 to OTG_HS
#if TUD_OPT_RHPORT == 0
  #define EP_MAX            EP_MAX_FS
  #define EP_FIFO_SIZE      EP_FIFO_SIZE_FS
  #define RHPORT_REGS_BASE  USB_OTG_FS_PERIPH_BASE
  #define RHPORT_IRQn       OTG_FS_IRQn

#else
  #define EP_MAX            EP_MAX_HS
  #define EP_FIFO_SIZE      EP_FIFO_SIZE_HS
  #define RHPORT_REGS_BASE  USB_OTG_HS_PERIPH_BASE
  #define RHPORT_IRQn       OTG_HS_IRQn
#endif

#define GLOBAL_BASE(_port)     ((USB_OTG_GlobalTypeDef*) RHPORT_REGS_BASE)
#define DEVICE_BASE(_port)     (USB_OTG_DeviceTypeDef *) (RHPORT_REGS_BASE + USB_OTG_DEVICE_BASE)
#define OUT_EP_BASE(_port)     (USB_OTG_OUTEndpointTypeDef *) (RHPORT_REGS_BASE + USB_OTG_OUT_ENDPOINT_BASE)
#define IN_EP_BASE(_port)      (USB_OTG_INEndpointTypeDef *) (RHPORT_REGS_BASE + USB_OTG_IN_ENDPOINT_BASE)
#define FIFO_BASE(_port, _x)   ((volatile uint32_t *) (RHPORT_REGS_BASE + USB_OTG_FIFO_BASE + (_x) * USB_OTG_FIFO_SIZE))

enum
{
  DCD_HIGH_SPEED        = 0, // Highspeed mode
  DCD_FULL_SPEED_USE_HS = 1, // Full speed in Highspeed port (probably with internal PHY)
  DCD_FULL_SPEED        = 3, // Full speed with internal PHY
};

static TU_ATTR_ALIGNED(4) uint32_t _setup_packet[2];

typedef struct {
  uint8_t * buffer;
  uint16_t total_len;
  uint16_t max_size;
} xfer_ctl_t;

typedef volatile uint32_t * usb_fifo_t;

//指示各个接口的状态，每个接口又IN和OUT两个端点
xfer_ctl_t xfer_status[EP_MAX][2];
#define XFER_CTL_BASE(_ep, _dir) &xfer_status[_ep][_dir]

//EP0 transfers are limited to 1 packet - larger sizes has to be split
static uint16_t ep0_pending[2];     // Index determines direction as tusb_dir_t type

//FIFO RAM allocation so far in words
static uint16_t _allocated_fifo_words;

// Setup the control endpoint 0.
static void bus_reset(uint8_t rhport)
{
  (void) rhport;

  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  tu_memclr(xfer_status, sizeof(xfer_status));

  for(uint8_t n = 0; n < EP_MAX; n++) {
    out_ep[n].DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
  }

  dev->DAINTMSK |= (1 << USB_OTG_DAINTMSK_OEPM_Pos) | (1 << USB_OTG_DAINTMSK_IEPM_Pos);
  dev->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
  dev->DIEPMSK |= USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM;

  // "USB Data FIFOs" section in reference manual
  // Peripheral FIFO architecture
  //
  // --------------- 320 or 1024 ( 1280 or 4096 bytes )
  // | IN FIFO MAX |
  // ---------------
  // |    ...      |
  // --------------- y + x + 16 + GRXFSIZ
  // | IN FIFO 2   |
  // --------------- x + 16 + GRXFSIZ
  // | IN FIFO 1   |
  // --------------- 16 + GRXFSIZ
  // | IN FIFO 0   |
  // --------------- GRXFSIZ
  // | OUT FIFO    |
  // | ( Shared )  |
  // --------------- 0
  //
  // According to "FIFO RAM allocation" section in RM, FIFO RAM are allocated as follows (each word 32-bits):
  // - Each EP IN needs at least max packet size, 16 words is sufficient for EP0 IN
  //
  // - All EP OUT shared a unique OUT FIFO which uses
  //   - 13 for setup packets + control words (up to 3 setup packets).
  //   - 1 for global NAK (not required/used here).
  //   - Largest-EPsize / 4 + 1. ( FS: 64 bytes, HS: 512 bytes). Recommended is  "2 x (Largest-EPsize/4) + 1"
  //   - 2 for each used OUT endpoint
  //
  //   Therefore GRXFSIZ = 13 + 1 + 1 + 2 x (Largest-EPsize/4) + 2 x EPOUTnum
  //   - FullSpeed (64 Bytes ): GRXFSIZ = 15 + 2 x  16 + 2 x EP_MAX = 47  + 2 x EP_MAX
  //   - Highspeed (512 bytes): GRXFSIZ = 15 + 2 x 128 + 2 x EP_MAX = 271 + 2 x EP_MAX
  //
  //   NOTE: Largest-EPsize & EPOUTnum is actual used endpoints in configuration. Since DCD has no knowledge
  //   of the overall picture yet. We will use the worst scenario: largest possible + EP_MAX
  //
  //   FIXME: for Isochronous, largest EP size can be 1023/1024 for FS/HS respectively. In addition if multiple ISO
  //   are enabled at least "2 x (Largest-EPsize/4) + 1" are recommended.  Maybe provide a macro for application to
  //   overwrite this.

#if TUD_OPT_HIGH_SPEED
  _allocated_fifo_words = 271 + 2*EP_MAX;
#else
  _allocated_fifo_words =  47 + 2*EP_MAX;
#endif

  usb_otg->GRXFSIZ = _allocated_fifo_words;

  // Control IN uses FIFO 0 with 64 bytes ( 16 32-bit word )
  usb_otg->DIEPTXF0_HNPTXFSIZ = (16 << USB_OTG_TX0FD_Pos) | _allocated_fifo_words;

  _allocated_fifo_words += 16;

  // TU_LOG2_INT(_allocated_fifo_words);

  // Fixed control EP0 size to 64 bytes
  in_ep[0].DIEPCTL &= ~(0x03 << USB_OTG_DIEPCTL_MPSIZ_Pos);
  xfer_status[0][TUSB_DIR_OUT].max_size = xfer_status[0][TUSB_DIR_IN].max_size = 64;

  out_ep[0].DOEPTSIZ |= (3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos);

  usb_otg->GINTMSK |= USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT;
}

// Set turn-around timeout according to link speed
extern uint32_t SystemCoreClock;
static void set_turnaround(USB_OTG_GlobalTypeDef * usb_otg, tusb_speed_t speed)
{
  usb_otg->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;

  if ( speed == TUSB_SPEED_HIGH )
  {
    // Use fixed 0x09 for Highspeed
    usb_otg->GUSBCFG |= (0x09 << USB_OTG_GUSBCFG_TRDT_Pos);
  }
  else
  {
    // Turnaround timeout depends on the MCU clock
    uint32_t turnaround;

    if ( SystemCoreClock >= 32000000U )
      turnaround = 0x6U;
    else if ( SystemCoreClock >= 27500000U )
      turnaround = 0x7U;
    else if ( SystemCoreClock >= 24000000U )
      turnaround = 0x8U;
    else if ( SystemCoreClock >= 21800000U )
      turnaround = 0x9U;
    else if ( SystemCoreClock >= 20000000U )
      turnaround = 0xAU;
    else if ( SystemCoreClock >= 18500000U )
      turnaround = 0xBU;
    else if ( SystemCoreClock >= 17200000U )
      turnaround = 0xCU;
    else if ( SystemCoreClock >= 16000000U )
      turnaround = 0xDU;
    else if ( SystemCoreClock >= 15000000U )
      turnaround = 0xEU;
    else
      turnaround = 0xFU;

    // Fullspeed depends on MCU clocks, but we will use 0x06 for 32+ Mhz
    usb_otg->GUSBCFG |= (turnaround << USB_OTG_GUSBCFG_TRDT_Pos);
  }
}

static tusb_speed_t get_speed(uint8_t rhport)
{
  (void) rhport;
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  uint32_t const enum_spd = (dev->DSTS & USB_OTG_DSTS_ENUMSPD_Msk) >> USB_OTG_DSTS_ENUMSPD_Pos;
  return (enum_spd == DCD_HIGH_SPEED) ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL;
}

static void set_speed(uint8_t rhport, tusb_speed_t speed)
{
  uint32_t bitvalue;

  if ( rhport == 1 )
  {
    bitvalue = ((TUSB_SPEED_HIGH == speed) ? DCD_HIGH_SPEED : DCD_FULL_SPEED_USE_HS);
  }
  else
  {
    bitvalue = DCD_FULL_SPEED;
  }

  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);

  // Clear and set speed bits
  dev->DCFG &= ~(3 << USB_OTG_DCFG_DSPD_Pos);
  dev->DCFG |= (bitvalue << USB_OTG_DCFG_DSPD_Pos);
}


#if defined(USB_HS_PHYC)
static bool USB_HS_PHYCInit(void)
{
  USB_HS_PHYC_GlobalTypeDef *usb_hs_phyc = (USB_HS_PHYC_GlobalTypeDef*) USB_HS_PHYC_CONTROLLER_BASE;

  // Enable LDO
  usb_hs_phyc->USB_HS_PHYC_LDO |= USB_HS_PHYC_LDO_ENABLE;

  // Wait until LDO ready
  while ( 0 == (usb_hs_phyc->USB_HS_PHYC_LDO & USB_HS_PHYC_LDO_STATUS) ) {}

  uint32_t phyc_pll = 0;

  // TODO Try to get HSE_VALUE from registers instead of depending CFLAGS
  switch ( HSE_VALUE )
  {
    case 12000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_12MHZ   ; break;
    case 12500000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_12_5MHZ ; break;
    case 16000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_16MHZ   ; break;
    case 24000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_24MHZ   ; break;
    case 25000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_25MHZ   ; break;
    case 32000000: phyc_pll = USB_HS_PHYC_PLL1_PLLSEL_Msk     ; break; // Value not defined in header
    default:
      TU_ASSERT(0);
  }
  usb_hs_phyc->USB_HS_PHYC_PLL = phyc_pll;

  // Control the tuning interface of the High Speed PHY
  // Use magic value (USB_HS_PHYC_TUNE_VALUE) from ST driver
  usb_hs_phyc->USB_HS_PHYC_TUNE |= 0x00000F13U;

  // Enable PLL internal PHY
  usb_hs_phyc->USB_HS_PHYC_PLL |= USB_HS_PHYC_PLL_PLLEN;

  // Original ST code has 2 ms delay for PLL stabilization.
  // Primitive test shows that more than 10 USB un/replug cycle showed no error with enumeration

  return true;
}
#endif

static void edpt_schedule_packets(uint8_t rhport, uint8_t const epnum, uint8_t const dir, uint16_t const num_packets, uint16_t total_bytes)
{
  (void) rhport;

  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  // EP0 is limited to one packet each xfer
  // We use multiple transaction of xfer->max_size length to get a whole transfer done
  if(epnum == 0) {
    xfer_ctl_t * const xfer = XFER_CTL_BASE(epnum, dir);
    total_bytes = tu_min16(ep0_pending[dir], xfer->max_size);
    ep0_pending[dir] -= total_bytes;
  }

  // IN and OUT endpoint xfers are interrupt-driven, we just schedule them here.
  if(dir == TUSB_DIR_IN) {
    // A full IN transfer (multiple packets, possibly) triggers XFRC.
    in_ep[epnum].DIEPTSIZ = (num_packets << USB_OTG_DIEPTSIZ_PKTCNT_Pos) |
        ((total_bytes << USB_OTG_DIEPTSIZ_XFRSIZ_Pos) & USB_OTG_DIEPTSIZ_XFRSIZ_Msk);

    in_ep[epnum].DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    // For ISO endpoint set correct odd/even bit for next frame.
    if ((in_ep[epnum].DIEPCTL & USB_OTG_DIEPCTL_EPTYP) == USB_OTG_DIEPCTL_EPTYP_0)
    {
      // Take odd/even bit from frame counter.
      uint32_t const odd_frame_now = (dev->DSTS & (1u << USB_OTG_DSTS_FNSOF_Pos));
      in_ep[epnum].DIEPCTL |= (odd_frame_now ? USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Msk : USB_OTG_DIEPCTL_SODDFRM_Msk);
    }
    // Enable fifo empty interrupt only if there are something to put in the fifo.
    if(total_bytes != 0) {
      dev->DIEPEMPMSK |= (1 << epnum);
    }
  } else {
    // A full OUT transfer (multiple packets, possibly) triggers XFRC.
    out_ep[epnum].DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT_Msk | USB_OTG_DOEPTSIZ_XFRSIZ);
    out_ep[epnum].DOEPTSIZ |= (num_packets << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
        ((total_bytes << USB_OTG_DOEPTSIZ_XFRSIZ_Pos) & USB_OTG_DOEPTSIZ_XFRSIZ_Msk);

    out_ep[epnum].DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
  }
}

/*------------------------------------------------------------------*/
/* Controller API
 *------------------------------------------------------------------*/
void dcd_init (uint8_t rhport)
{
  // Programming model begins in the last section of the chapter on the USB
  // peripheral in each Reference Manual.

  volatile USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);

  // No HNP/SRP (no OTG support), program timeout later.
  if ( rhport == 1 )
  {
    // On selected MCUs HS port1 can be used with external PHY via ULPI interface

    // deactivate internal PHY
    usb_otg->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;
    // Init The UTMI Interface
    usb_otg->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

    // Select default internal VBUS Indicator and Drive for ULPI
    usb_otg->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);

#if defined(USB_HS_PHYC)
    // Highspeed with embedded UTMI PHYC

    // Select UTMI Interface
    usb_otg->GUSBCFG &= ~USB_OTG_GUSBCFG_ULPI_UTMI_SEL;
    usb_otg->GCCFG |= USB_OTG_GCCFG_PHYHSEN;

    // Enables control of a High Speed USB PHY
    USB_HS_PHYCInit();
#endif
  } else
  {
    //Enable internal PHY
    usb_otg->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
  }

  // Reset core after selecting PHY
  // Wait AHB IDLE, reset then wait until it is cleared
  while ((usb_otg->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U) {}
  usb_otg->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
  while ((usb_otg->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST) {}

  // Restart PHY clock
  *((volatile uint32_t *)(RHPORT_REGS_BASE + USB_OTG_PCGCCTL_BASE)) = 0;

  // Clear all interrupts
  usb_otg->GINTSTS |= usb_otg->GINTSTS;

  // Required as part of core initialization.
  // TODO: How should mode mismatch be handled? It will cause
  // the core to stop working/require reset.
  usb_otg->GINTMSK |= USB_OTG_GINTMSK_OTGINT | USB_OTG_GINTMSK_MMISM;

  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);

  // If USB host misbehaves during status portion of control xfer
  // (non zero-length packet), send STALL back and discard.
  dev->DCFG |=  USB_OTG_DCFG_NZLSOHSK;

  set_speed(rhport, TUD_OPT_HIGH_SPEED ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL);

  // Enable internal USB transceiver.
  if ( rhport == 0 ) usb_otg->GCCFG |= USB_OTG_GCCFG_PWRDWN;

  usb_otg->GINTMSK |= USB_OTG_GINTMSK_USBRST   | USB_OTG_GINTMSK_ENUMDNEM |
                      USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM     |
                      USB_OTG_GINTMSK_RXFLVLM  | (USE_SOF ? USB_OTG_GINTMSK_SOFM : 0);

  // Enable global interrupt
  usb_otg->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

  dcd_connect(rhport);
}

void dcd_int_enable (uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ(RHPORT_IRQn);
}

void dcd_int_disable (uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ(RHPORT_IRQn);
}

void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
	dev->DCFG &= ~(USB_OTG_DCFG_DAD);
  dev->DCFG |= (dev_addr << USB_OTG_DCFG_DAD_Pos) & USB_OTG_DCFG_DAD_Msk;

  // Response with status after changing device address
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
}


void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
/*
	应用程序使用该位向USB_OTG模块发出执行软断开的信号。
  该位置1时,主机不会看到设备已连接,且该设备也不会接收USB上的信号。
  在应用程序将此位清零之前，模块会保持断开状态。
*/
  dev->DCTL &= ~USB_OTG_DCTL_SDIS;
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);

  dev->DCTL |= USB_OTG_DCTL_SDIS;
}


/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  uint8_t const epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
  uint8_t const dir   = tu_edpt_dir(desc_edpt->bEndpointAddress);

  TU_ASSERT(epnum < EP_MAX);

  if (desc_edpt->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS)
  {
    TU_ASSERT(desc_edpt->wMaxPacketSize.size <= (get_speed(rhport) == TUSB_SPEED_HIGH ? 1024 : 1023));
  }
  else
  {
    TU_ASSERT(desc_edpt->wMaxPacketSize.size <= (get_speed(rhport) == TUSB_SPEED_HIGH ? 512 : 64));
  }

  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
  xfer->max_size = desc_edpt->wMaxPacketSize.size;

  if(dir == TUSB_DIR_OUT)
  {
    out_ep[epnum].DOEPCTL |= (1 << USB_OTG_DOEPCTL_USBAEP_Pos) |
                             (desc_edpt->bmAttributes.xfer << USB_OTG_DOEPCTL_EPTYP_Pos) |
                             (desc_edpt->wMaxPacketSize.size << USB_OTG_DOEPCTL_MPSIZ_Pos);

    dev->DAINTMSK |= (1 << (USB_OTG_DAINTMSK_OEPM_Pos + epnum));
  }
  else
  {
    // "USB Data FIFOs" section in reference manual
    // Peripheral FIFO architecture
    //
    // --------------- 320 or 1024 ( 1280 or 4096 bytes )
    // | IN FIFO MAX |
    // ---------------
    // |    ...      |
    // --------------- y + x + 16 + GRXFSIZ
    // | IN FIFO 2   |
    // --------------- x + 16 + GRXFSIZ
    // | IN FIFO 1   |
    // --------------- 16 + GRXFSIZ
    // | IN FIFO 0   |
    // --------------- GRXFSIZ
    // | OUT FIFO    |
    // | ( Shared )  |
    // --------------- 0
    //
    // In FIFO is allocated by following rules:
    // - IN EP 1 gets FIFO 1, IN EP "n" gets FIFO "n".
    // - Offset: allocated so far
    // - Size
    //    - Interrupt is EPSize
    //    - Bulk/ISO is max(EPSize, remaining-fifo / non-opened-EPIN)

    uint16_t const fifo_remaining = EP_FIFO_SIZE/4 - _allocated_fifo_words;
    uint16_t fifo_size = desc_edpt->wMaxPacketSize.size / 4;

    if ( desc_edpt->bmAttributes.xfer != TUSB_XFER_INTERRUPT )
    {
      uint8_t opened = 0;
      for(uint8_t i = 0; i < EP_MAX; i++)
      {
        if ( (i != epnum) && (xfer_status[i][TUSB_DIR_IN].max_size > 0) ) opened++;
      }

      // EP Size or equally divided of remaining whichever is larger
      fifo_size = tu_max16(fifo_size, fifo_remaining / (EP_MAX - opened));
    }

    // FIFO overflows, we probably need a better allocating scheme
    TU_ASSERT(fifo_size <= fifo_remaining);

    // DIEPTXF starts at FIFO #1.
    // Both TXFD and TXSA are in unit of 32-bit words.
    usb_otg->DIEPTXF[epnum - 1] = (fifo_size << USB_OTG_DIEPTXF_INEPTXFD_Pos) | _allocated_fifo_words;

    _allocated_fifo_words += fifo_size;

    in_ep[epnum].DIEPCTL |= (1 << USB_OTG_DIEPCTL_USBAEP_Pos) |
                            (epnum << USB_OTG_DIEPCTL_TXFNUM_Pos) |
                            (desc_edpt->bmAttributes.xfer << USB_OTG_DIEPCTL_EPTYP_Pos) |
                            (desc_edpt->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS ? USB_OTG_DOEPCTL_SD0PID_SEVNFRM : 0) |
                            (desc_edpt->wMaxPacketSize.size << USB_OTG_DIEPCTL_MPSIZ_Pos);

    dev->DAINTMSK |= (1 << (USB_OTG_DAINTMSK_IEPM_Pos + epnum));
  }

  return true;
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, dir);
  xfer->buffer      = buffer;
  xfer->total_len   = total_bytes;

  // EP0 can only handle one packet
  if(epnum == 0) {
    ep0_pending[dir] = total_bytes;
    // Schedule the first transaction for EP0 transfer
    edpt_schedule_packets(rhport, epnum, dir, 1, ep0_pending[dir]);
    return true;
  }

  uint16_t num_packets = (total_bytes / xfer->max_size);
  uint8_t const short_packet_size = total_bytes % xfer->max_size;

  // Zero-size packet is special case.
  if(short_packet_size > 0 || (total_bytes == 0)) {
    num_packets++;
  }

  // Schedule packets to be sent within interrupt
  edpt_schedule_packets(rhport, epnum, dir, num_packets, total_bytes);

  return true;
}

// TODO: The logic for STALLing and disabling an endpoint is very similar
// (send STALL versus NAK handshakes back). Refactor into resuable function.
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  if(dir == TUSB_DIR_IN) {
    // Only disable currently enabled non-control endpoint
    if ( (epnum == 0) || !(in_ep[epnum].DIEPCTL & USB_OTG_DIEPCTL_EPENA) ){
      in_ep[epnum].DIEPCTL |= (USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_STALL);
    } else {
      // Stop transmitting packets and NAK IN xfers.
      in_ep[epnum].DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
      while((in_ep[epnum].DIEPINT & USB_OTG_DIEPINT_INEPNE) == 0);

      // Disable the endpoint.
      in_ep[epnum].DIEPCTL |= (USB_OTG_DIEPCTL_STALL | USB_OTG_DIEPCTL_EPDIS);
      while((in_ep[epnum].DIEPINT & USB_OTG_DIEPINT_EPDISD_Msk) == 0);
      in_ep[epnum].DIEPINT = USB_OTG_DIEPINT_EPDISD;
    }

    // Flush the FIFO, and wait until we have confirmed it cleared.
    usb_otg->GRSTCTL |= (epnum << USB_OTG_GRSTCTL_TXFNUM_Pos);
    usb_otg->GRSTCTL |= USB_OTG_GRSTCTL_TXFFLSH;
    while((usb_otg->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH_Msk) != 0);
  } else {
    // Only disable currently enabled non-control endpoint
    if ( (epnum == 0) || !(out_ep[epnum].DOEPCTL & USB_OTG_DOEPCTL_EPENA) ){
      out_ep[epnum].DOEPCTL |= USB_OTG_DOEPCTL_STALL;
    } else {
      // Asserting GONAK is required to STALL an OUT endpoint.
      // Simpler to use polling here, we don't use the "B"OUTNAKEFF interrupt
      // anyway, and it can't be cleared by user code. If this while loop never
      // finishes, we have bigger problems than just the stack.
      dev->DCTL |= USB_OTG_DCTL_SGONAK;
      while((usb_otg->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF_Msk) == 0);

      // Ditto here- disable the endpoint.
      out_ep[epnum].DOEPCTL |= (USB_OTG_DOEPCTL_STALL | USB_OTG_DOEPCTL_EPDIS);
      while((out_ep[epnum].DOEPINT & USB_OTG_DOEPINT_EPDISD_Msk) == 0);
      out_ep[epnum].DOEPINT = USB_OTG_DOEPINT_EPDISD;

      // Allow other OUT endpoints to keep receiving.
      dev->DCTL |= USB_OTG_DCTL_CGONAK;
    }
  }
}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  if(dir == TUSB_DIR_IN) {
    in_ep[epnum].DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;

    uint8_t eptype = (in_ep[epnum].DIEPCTL & USB_OTG_DIEPCTL_EPTYP_Msk) >> USB_OTG_DIEPCTL_EPTYP_Pos;
    // Required by USB spec to reset DATA toggle bit to DATA0 on interrupt and bulk endpoints.
    if(eptype == 2 || eptype == 3) {
      in_ep[epnum].DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
    }
  } else {
    out_ep[epnum].DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;

    uint8_t eptype = (out_ep[epnum].DOEPCTL & USB_OTG_DOEPCTL_EPTYP_Msk) >> USB_OTG_DOEPCTL_EPTYP_Pos;
    // Required by USB spec to reset DATA toggle bit to DATA0 on interrupt and bulk endpoints.
    if(eptype == 2 || eptype == 3) {
      out_ep[epnum].DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
    }
  }
}

/*------------------------------------------------------------------*/

//Read a single data packet from receive FIFO
static void read_fifo_packet(uint8_t rhport, uint8_t * dst, uint16_t len)
{
  (void) rhport;

  usb_fifo_t rx_fifo = FIFO_BASE(rhport, 0);

  // Reading full available 32 bit words from fifo
  uint16_t full_words = len >> 2;
  for(uint16_t i = 0; i < full_words; i++) {
    uint32_t tmp = *rx_fifo;
    dst[0] = tmp & 0x000000FF;
    dst[1] = (tmp & 0x0000FF00) >> 8;
    dst[2] = (tmp & 0x00FF0000) >> 16;
    dst[3] = (tmp & 0xFF000000) >> 24;
    dst += 4;
  }

  // Read the remaining 1-3 bytes from fifo
  uint8_t bytes_rem = len & 0x03;
  if(bytes_rem != 0) {
    uint32_t tmp = *rx_fifo;
    dst[0] = tmp & 0x000000FF;
    if(bytes_rem > 1) {
      dst[1] = (tmp & 0x0000FF00) >> 8;
    }
    if(bytes_rem > 2) {
      dst[2] = (tmp & 0x00FF0000) >> 16;
    }
  }
}

// Write a single data packet to EPIN FIFO
static void write_fifo_packet(uint8_t rhport, uint8_t fifo_num, uint8_t * src, uint16_t len)
{
  (void) rhport;

  usb_fifo_t tx_fifo = FIFO_BASE(rhport, fifo_num);

  // Pushing full available 32 bit words to fifo
  uint16_t full_words = len >> 2;
  for(uint16_t i = 0; i < full_words; i++){
    *tx_fifo = (src[3] << 24) | (src[2] << 16) | (src[1] << 8) | src[0];
    src += 4;
  }

  // Write the remaining 1-3 bytes into fifo
  uint8_t bytes_rem = len & 0x03;
  if(bytes_rem){
    uint32_t tmp_word = 0;
    tmp_word |= src[0];
    if(bytes_rem > 1){
      tmp_word |= src[1] << 8;
    }
    if(bytes_rem > 2){
      tmp_word |= src[2] << 16;
    }
    *tx_fifo = tmp_word;
  }
}
static void handle_rxflvl_ints(uint8_t rhport, USB_OTG_OUTEndpointTypeDef * out_ep) 
{
  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  usb_fifo_t rx_fifo = FIFO_BASE(rhport, 0);
  uint32_t ctl_word = usb_otg->GRXSTSP;

	//指示接收的数据包的状态
  uint8_t pktsts = (ctl_word & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos;
	//指示当前接收的数据包所属的通道编号
  uint8_t epnum = (ctl_word &  USB_OTG_GRXSTSP_EPNUM_Msk) >>  USB_OTG_GRXSTSP_EPNUM_Pos;
	//指示接收的IN数据包的字节数
  uint16_t bcnt = (ctl_word & USB_OTG_GRXSTSP_BCNT_Msk) >> USB_OTG_GRXSTSP_BCNT_Pos;

  switch(pktsts)
	{
    case 0x01: // Global OUT NAK (Interrupt)
    break;

    case 0x02: //接收到OUT数据包(主机发过来的数据包)
    {
      xfer_ctl_t * xfer = XFER_CTL_BASE(epnum, TUSB_DIR_OUT);			
      // Read packet off RxFIFO
      read_fifo_packet(rhport, xfer->buffer, bcnt);
      // Increment pointer to xfer data
      xfer->buffer += bcnt;
      // Truncate transfer length in case of short packet
      if(bcnt < xfer->max_size) {
        xfer->total_len -= (out_ep[epnum].DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ_Msk) >> USB_OTG_DOEPTSIZ_XFRSIZ_Pos;
        if(epnum == 0) {
          xfer->total_len -= ep0_pending[TUSB_DIR_OUT];
          ep0_pending[TUSB_DIR_OUT] = 0;
        }
      }
    }
    break;

    case 0x03: //OUT传输完成（触发中断）
      break;

    case 0x04: //SETUP事务完成（触发中断）
      out_ep[epnum].DOEPTSIZ |= (3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos);
    break;

    case 0x06: //接收到SETUP数据包
		{
      // We can receive up to three setup packets in succession, but
      // only the last one is valid.
      _setup_packet[0] = (* rx_fifo);
      _setup_packet[1] = (* rx_fifo);
      break;
		}
		
    default: //保留
      TU_BREAKPOINT();
    break;
  }
}

static void handle_epout_ints(uint8_t rhport, USB_OTG_DeviceTypeDef * dev, USB_OTG_OUTEndpointTypeDef * out_ep)
{
  // DAINT for a given EP clears when DOEPINTx is cleared.
  // OEPINT will be cleared when DAINT's out bits are cleared.
  for(uint8_t n = 0; n < EP_MAX; n++) 
	{
    xfer_ctl_t * xfer = XFER_CTL_BASE(n, TUSB_DIR_OUT);
    if(dev->DAINT & (1 << (USB_OTG_DAINT_OEPINT_Pos + n))) //确定是哪一个OUT端点
		{
      //SETUP阶段完成,可以对接收到的 SETUP 数据包进行解码,仅适用于控制 OUT 端点
      if(out_ep[n].DOEPINT & USB_OTG_DOEPINT_STUP) 
			{
        out_ep[n].DOEPINT =  USB_OTG_DOEPINT_STUP;
        dcd_event_setup_received(rhport, (uint8_t*) &_setup_packet[0], true);
      }
      //传输完成中断
      if(out_ep[n].DOEPINT & USB_OTG_DOEPINT_XFRC) 
			{
        out_ep[n].DOEPINT = USB_OTG_DOEPINT_XFRC;
        // EP0 can only handle one packet
        if((n == 0) && ep0_pending[TUSB_DIR_OUT]) 
				{
          // Schedule another packet to be received.
          edpt_schedule_packets(rhport, n, TUSB_DIR_OUT, 1, ep0_pending[TUSB_DIR_OUT]);
        } 
				else 
				{
          dcd_event_xfer_complete(rhport, n, xfer->total_len, XFER_RESULT_SUCCESS, true);
				}
      }
    }
  }
}

static void handle_epin_ints(uint8_t rhport, USB_OTG_DeviceTypeDef * dev, USB_OTG_INEndpointTypeDef * in_ep) {
  // DAINT for a given EP clears when DIEPINTx is cleared.
  // IEPINT will be cleared when DAINT's out bits are cleared.
  for ( uint8_t n = 0; n < EP_MAX; n++ )
  {
    xfer_ctl_t *xfer = XFER_CTL_BASE(n, TUSB_DIR_IN);

    if( dev->DAINT & (1 << (USB_OTG_DAINT_IEPINT_Pos + n)) )//确定是哪一个IN端点
    {
      //IN传输完成中断
      if ( in_ep[n].DIEPINT & USB_OTG_DIEPINT_XFRC )
      {
        in_ep[n].DIEPINT = USB_OTG_DIEPINT_XFRC;
        // EP0 can only handle one packet
        if((n == 0) && ep0_pending[TUSB_DIR_IN]) {
          // Schedule another packet to be transmitted.
          edpt_schedule_packets(rhport, n, TUSB_DIR_IN, 1, ep0_pending[TUSB_DIR_IN]);
        } 
				else 
				{	
          dcd_event_xfer_complete(rhport, n | TUSB_DIR_IN_MASK, xfer->total_len, XFER_RESULT_SUCCESS, true);
				}
      }
      //发送 FIFO 为空
      if ( (in_ep[n].DIEPINT & USB_OTG_DIEPINT_TXFE) && (dev->DIEPEMPMSK & (1 << n)) )
      {
        // DIEPINT's TXFE bit is read-only, software cannot clear it.
        // It will only be cleared by hardware when written bytes is more than
        // - 64 bytes or
        // - Half of TX FIFO size (configured by DIEPTXF)

        uint16_t remaining_packets = (in_ep[n].DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT_Msk) >> USB_OTG_DIEPTSIZ_PKTCNT_Pos;

        // Process every single packet (only whole packets can be written to fifo)
        for(uint16_t i = 0; i < remaining_packets; i++){
          uint16_t remaining_bytes = (in_ep[n].DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ_Msk) >> USB_OTG_DIEPTSIZ_XFRSIZ_Pos;
          // Packet can not be larger than ep max size
          uint16_t packet_size = tu_min16(remaining_bytes, xfer->max_size);

          // It's only possible to write full packets into FIFO. Therefore DTXFSTS register of current
          // EP has to be checked if the buffer can take another WHOLE packet
          if(packet_size > ((in_ep[n].DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) << 2)){
            break;
          }

          // Push packet to Tx-FIFO
          write_fifo_packet(rhport, n, xfer->buffer, packet_size);

          // Increment pointer to xfer data
          xfer->buffer += packet_size;
        }

        // Turn off TXFE if all bytes are written.
        if (((in_ep[n].DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ_Msk) >> USB_OTG_DIEPTSIZ_XFRSIZ_Pos) == 0)
        {
          dev->DIEPEMPMSK &= ~(1 << n);
        }
      }
    }
  }
}


void usb_handler(uint8_t rhport)
{
  USB_OTG_GlobalTypeDef * usb_otg = GLOBAL_BASE(rhport);
  USB_OTG_DeviceTypeDef * dev = DEVICE_BASE(rhport);
  USB_OTG_OUTEndpointTypeDef * out_ep = OUT_EP_BASE(rhport);
  USB_OTG_INEndpointTypeDef * in_ep = IN_EP_BASE(rhport);

  uint32_t int_status = usb_otg->GINTSTS;

	//复位中断
  if(int_status & USB_OTG_GINTSTS_USBRST)
	{
    usb_otg->GINTSTS = USB_OTG_GINTSTS_USBRST;
    bus_reset(rhport);
  }
	
	/*
		该位置 1 时,指示速率枚举已完成。
		应用程序必须读取OTG_DSTS寄存器来获取枚
		举速率。
	*/
  if(int_status & USB_OTG_GINTSTS_ENUMDNE)
	{
    usb_otg->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
    tusb_speed_t const speed = get_speed(rhport);
		// USB周转时间,以 PHY 时钟数为单位设置周转时间,决于应用程序 AHB 频率
    set_turnaround(usb_otg, speed);
    dcd_event_bus_reset(rhport, speed, true);
  }

	//USB挂起中断
  if(int_status & USB_OTG_GINTSTS_USBSUSP)
  {				
    usb_otg->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
  }
	
	//在设备模式下,当USB总线上检测到恢复信号时,将触发该中断。
  if(int_status & USB_OTG_GINTSTS_WKUINT)
  {
    usb_otg->GINTSTS = USB_OTG_GINTSTS_WKUINT;
    dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
  }

	//OTG中断,此位只读
  if(int_status & USB_OTG_GINTSTS_OTGINT)
  {
    uint32_t const otg_int = usb_otg->GOTGINT;
    if (otg_int & USB_OTG_GOTGINT_SEDET)
    {
      dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
    }
    usb_otg->GOTGINT = otg_int;
  }

#if USE_SOF
  if(int_status & USB_OTG_GINTSTS_SOF) {
    usb_otg->GINTSTS = USB_OTG_GINTSTS_SOF;
    dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
  }
#endif

  //Rx FIFO非空中断,此位只读
	if(int_status & USB_OTG_GINTSTS_RXFLVL) 
	{
    // 当读取Rx FIFO时,屏蔽RXFLVL中断
    usb_otg->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;

    // Loop until all available packets were handled
    do 
		{
      handle_rxflvl_ints(rhport, out_ep); 
      int_status = usb_otg->GINTSTS;
    } while(int_status & USB_OTG_GINTSTS_RXFLVL);

    usb_otg->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
	}

  //OUT端点中断,只读位,device接收
  if(int_status & USB_OTG_GINTSTS_OEPINT)
	{		
    handle_epout_ints(rhport, dev, out_ep);		
  }
  //IN端点中断,只读位,device发送
  if(int_status & USB_OTG_GINTSTS_IEPINT) 
	{				
    handle_epin_ints(rhport, dev, in_ep);
  }
}

#endif


/*************************************************************************
**************************************************************************
**************************************************************************/
#if tusb_c
static bool _initialized = false;
bool tusb_init(void)
{
	if (_initialized) 
		return true;
#if TUSB_OPT_HOST_ENABLED
	TU_ASSERT( usbh_init() ); // init host stack
#endif
#if TUSB_OPT_DEVICE_ENABLED
	TU_ASSERT ( tud_init() ); // init device stack
#endif
	_initialized = true;
	return TUSB_ERROR_NONE;
}

bool tusb_inited(void)
{
	return _initialized;
}
#endif


/*************************************************************************
**************************************************************************
**************************************************************************/
#if usb_descriptors_c

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]         HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) )
//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_MSC,
  ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define EPNUM_MSC_OUT     0x03
#define EPNUM_MSC_IN      0x83

uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

  // Interface number, string index, EP Out & EP In address, EP size
  TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

#if TUD_OPT_HIGH_SPEED
uint8_t const desc_hs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 512),

  // Interface number, string index, EP Out & EP In address, EP size
  TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 512),
};
#endif

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
  // Although we are highspeed, host may be fullspeed.
  return (tud_speed_get() == TUSB_SPEED_HIGH) ?  desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr [] =
{
  (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
  "ACFLy",                     // 1: Manufacturer
  "ACFLy Device",              // 2: Product
  "123456",                      // 3: Serials, should use chip ID
  "ACFLy CDC",                 // 4: CDC Interface
  "ACFLy MSC",                 // 5: MSC Interface
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  if ( index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }else
  {
    // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
    // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = strlen(str);
    if ( chr_count > 31 ) chr_count = 31;

    // Convert ASCII string into UTF-16
    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}	
#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if usb_contronl_c

#if CFG_TUSB_DEBUG >= 2
extern void usbd_driver_print_control_complete_name(bool (*control_complete) (uint8_t, tusb_control_request_t const *));
#endif

enum
{
  EDPT_CTRL_OUT = 0x00,
  EDPT_CTRL_IN  = 0x80
};

typedef struct
{
  tusb_control_request_t request;

  uint8_t* buffer;
  uint16_t data_len;
  uint16_t total_xferred;

  bool (*complete_cb) (uint8_t, tusb_control_request_t const *);
} usbd_control_xfer_t;

static usbd_control_xfer_t _ctrl_xfer;

CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN
static uint8_t _usbd_ctrl_buf[CFG_TUD_ENDPOINT0_SIZE];

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+
// Queue ZLP status transaction
static inline bool _status_stage_xact(uint8_t rhport, tusb_control_request_t const * request)
{
  // Opposite to endpoint in Data Phase
  uint8_t const ep_addr = request->bmRequestType_bit.direction ? EDPT_CTRL_OUT : EDPT_CTRL_IN;

  TU_LOG2("  Queue EP %02X with zlp Status\r\n", ep_addr);

  // status direction is reversed to one in the setup packet
  // Note: Status must always be DATA1
  return dcd_edpt_xfer(rhport, ep_addr, NULL, 0);
}

// Status phase
bool tud_control_status(uint8_t rhport, tusb_control_request_t const * request)
{
  _ctrl_xfer.request       = (*request);
  _ctrl_xfer.buffer        = NULL;
  _ctrl_xfer.total_xferred = 0;
  _ctrl_xfer.data_len      = 0;

  return _status_stage_xact(rhport, request);
}

// Queue a transaction in Data Stage
// Each transaction has up to Endpoint0's max packet size.
// This function can also transfer an zero-length packet
static bool _data_stage_xact(uint8_t rhport)
{
  uint16_t const xact_len = tu_min16(_ctrl_xfer.data_len - _ctrl_xfer.total_xferred, CFG_TUD_ENDPOINT0_SIZE);

  uint8_t ep_addr = EDPT_CTRL_OUT;

  if ( _ctrl_xfer.request.bmRequestType_bit.direction == TUSB_DIR_IN )
  {
    ep_addr = EDPT_CTRL_IN;
    if ( xact_len ) memcpy(_usbd_ctrl_buf, _ctrl_xfer.buffer, xact_len);
  }

  TU_LOG2("  Queue EP %02X with %u bytes\r\n", ep_addr, xact_len);

  return dcd_edpt_xfer(rhport, ep_addr, xact_len ? _usbd_ctrl_buf : NULL, xact_len);
}

// Transmit data to/from the control endpoint.
// If the request's wLength is zero, a status packet is sent instead.
bool tud_control_xfer(uint8_t rhport, tusb_control_request_t const * request, void* buffer, uint16_t len)
{
  _ctrl_xfer.request       = (*request);
  _ctrl_xfer.buffer        = (uint8_t*) buffer;
  _ctrl_xfer.total_xferred = 0U;
  _ctrl_xfer.data_len      = tu_min16(len, request->wLength);
  
  if (request->wLength > 0U)
  {
    if(_ctrl_xfer.data_len > 0U)
    {
      TU_ASSERT(buffer);
    }

//    TU_LOG2("  Control total data length is %u bytes\r\n", _ctrl_xfer.data_len);

    // Data stage
    TU_ASSERT( _data_stage_xact(rhport) );
  }
  else
  {
    // Status stage
    TU_ASSERT( _status_stage_xact(rhport, request) );
  }

  return true;
}

//--------------------------------------------------------------------+
// USBD API
//--------------------------------------------------------------------+
void usbd_control_reset(void)
{
  tu_varclr(&_ctrl_xfer);
}

// TODO may find a better way
void usbd_control_set_complete_callback( bool (*fp) (uint8_t, tusb_control_request_t const * ) )
{
  _ctrl_xfer.complete_cb = fp;
}

// useful for dcd_set_address where DCD is responsible for status response
void usbd_control_set_request(tusb_control_request_t const *request)
{
  _ctrl_xfer.request       = (*request);
  _ctrl_xfer.buffer        = NULL;
  _ctrl_xfer.total_xferred = 0;
  _ctrl_xfer.data_len      = 0;
}

// callback when a transaction complete on
// - DATA stage of control endpoint or
// - Status stage
bool usbd_control_xfer_cb (uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) result;

  // Endpoint Address is opposite to direction bit, this is Status Stage complete event
  if ( tu_edpt_dir(ep_addr) != _ctrl_xfer.request.bmRequestType_bit.direction )
  {
    TU_ASSERT(0 == xferred_bytes);
    if (dcd_edpt0_status_complete) dcd_edpt0_status_complete(rhport, &_ctrl_xfer.request);
    return true;
  }

  if ( _ctrl_xfer.request.bmRequestType_bit.direction == TUSB_DIR_OUT )
  {
    TU_VERIFY(_ctrl_xfer.buffer);
    memcpy(_ctrl_xfer.buffer, _usbd_ctrl_buf, xferred_bytes);
  }

  _ctrl_xfer.total_xferred += xferred_bytes;
  _ctrl_xfer.buffer += xferred_bytes;

  // Data Stage is complete when all request's length are transferred or
  // a short packet is sent including zero-length packet.
  if ( (_ctrl_xfer.request.wLength == _ctrl_xfer.total_xferred) || (xferred_bytes < CFG_TUD_ENDPOINT0_SIZE) )
  {
    // DATA stage is complete
    bool is_ok = true;

    // invoke complete callback if set
    // callback can still stall control in status phase e.g out data does not make sense
    if ( _ctrl_xfer.complete_cb )
    {
      #if CFG_TUSB_DEBUG >= 2
      usbd_driver_print_control_complete_name(_ctrl_xfer.complete_cb);
      #endif

      is_ok = _ctrl_xfer.complete_cb(rhport, &_ctrl_xfer.request);
    }

    if ( is_ok )
    {
      // Send status
      TU_ASSERT( _status_stage_xact(rhport, &_ctrl_xfer.request) );
    }else
    {
      // Stall both IN and OUT control endpoint
      dcd_edpt_stall(rhport, EDPT_CTRL_OUT);
      dcd_edpt_stall(rhport, EDPT_CTRL_IN);
    }
  }
  else
  {
    // More data to transfer
    TU_ASSERT( _data_stage_xact(rhport) );
  }

  return true;
}

#endif
/*************************************************************************
**************************************************************************
**************************************************************************/
#if usbd_c

#ifndef CFG_TUD_TASK_QUEUE_SZ
#define CFG_TUD_TASK_QUEUE_SZ   100
#endif

usbd_device_t _usbd_dev;

#if CFG_TUSB_OS != OPT_OS_NONE
static osal_mutex_def_t _ubsd_mutexdef[2];
static osal_mutex_t _usbd_mutex[2];
#endif
enum { DRVID_INVALID = 0xFFu };

#if CFG_TUSB_DEBUG >= 2
  #define DRIVER_NAME(_name)    .name = _name,
#else
  #define DRIVER_NAME(_name)
#endif

static usbd_class_driver_t const _usbd_driver[] =
{
  #if CFG_TUD_CDC
  {
      DRIVER_NAME("CDC")
      .init             = cdcd_init,
      .reset            = cdcd_reset,
      .open             = cdcd_open,
      .control_request  = cdcd_control_request,
      .control_complete = cdcd_control_complete,
      .xfer_cb          = cdcd_xfer_cb,
      .sof              = NULL
  },
  #endif

  #if CFG_TUD_MSC
  {
      DRIVER_NAME("MSC")
      .init             = mscd_init,
      .reset            = mscd_reset,
      .open             = mscd_open,
      .control_request  = mscd_control_request,
      .control_complete = mscd_control_complete,
      .xfer_cb          = mscd_xfer_cb,
      .sof              = NULL
  },
  #endif

  #if CFG_TUD_HID
  {
      DRIVER_NAME("HID")
      .init             = hidd_init,
      .reset            = hidd_reset,
      .open             = hidd_open,
      .control_request  = hidd_control_request,
      .control_complete = hidd_control_complete,
      .xfer_cb          = hidd_xfer_cb,
      .sof              = NULL
  },
  #endif

  #if CFG_TUD_MIDI
  {
      DRIVER_NAME("MIDI")
      .init             = midid_init,
      .open             = midid_open,
      .reset            = midid_reset,
      .control_request  = midid_control_request,
      .control_complete = midid_control_complete,
      .xfer_cb          = midid_xfer_cb,
      .sof              = NULL
  },
  #endif

  #if CFG_TUD_VENDOR
  {
      DRIVER_NAME("VENDOR")
      .init             = vendord_init,
      .reset            = vendord_reset,
      .open             = vendord_open,
      .control_request  = tud_vendor_control_request_cb,
      .control_complete = tud_vendor_control_complete_cb,
      .xfer_cb          = vendord_xfer_cb,
      .sof              = NULL
  },
  #endif

  #if CFG_TUD_USBTMC
  {
      DRIVER_NAME("TMC")
      .init             = usbtmcd_init_cb,
      .reset            = usbtmcd_reset_cb,
      .open             = usbtmcd_open_cb,
      .control_request  = usbtmcd_control_request_cb,
      .control_complete = usbtmcd_control_complete_cb,
      .xfer_cb          = usbtmcd_xfer_cb,
      .sof              = NULL
  },
  #endif

  #if CFG_TUD_DFU_RT
  {
      DRIVER_NAME("DFU-RT")
      .init             = dfu_rtd_init,
      .reset            = dfu_rtd_reset,
      .open             = dfu_rtd_open,
      .control_request  = dfu_rtd_control_request,
      .control_complete = dfu_rtd_control_complete,
      .xfer_cb          = dfu_rtd_xfer_cb,
      .sof              = NULL
  },
  #endif

  #if CFG_TUD_NET
  {
      DRIVER_NAME("NET")
      .init             = netd_init,
      .reset            = netd_reset,
      .open             = netd_open,
      .control_request  = netd_control_request,
      .control_complete = netd_control_complete,
      .xfer_cb          = netd_xfer_cb,
      .sof              = NULL,
  },
  #endif

  #if CFG_TUD_BTH
  {
      DRIVER_NAME("BTH")
      .init             = btd_init,
      .reset            = btd_reset,
      .open             = btd_open,
      .control_request  = btd_control_request,
      .control_complete = btd_control_complete,
      .xfer_cb          = btd_xfer_cb,
      .sof              = NULL
  },
  #endif
};

enum { BUILTIN_DRIVER_COUNT = TU_ARRAY_SIZE(_usbd_driver) };

// Additional class drivers implemented by application
static usbd_class_driver_t const * _app_driver = NULL;
static uint8_t _app_driver_count = 0;

// virtually joins built-in and application drivers together.
// Application is positioned first to allow overwriting built-in ones.
static inline usbd_class_driver_t const * get_driver(uint8_t drvid)
{
  // Application drivers
  if( usbd_app_driver_get_cb )
  {
    if ( drvid < _app_driver_count ) return &_app_driver[drvid];
    drvid -= _app_driver_count;
  }
  // Built-in drivers
  if(drvid < BUILTIN_DRIVER_COUNT) return &_usbd_driver[drvid];
  return NULL;
}

#define TOTAL_DRIVER_COUNT    (_app_driver_count + BUILTIN_DRIVER_COUNT)

//--------------------------------------------------------------------+
// DCD Event
//--------------------------------------------------------------------+
OSAL_QUEUE_DEF(OPT_MODE_DEVICE, _usbd_qdef, CFG_TUD_TASK_QUEUE_SZ, dcd_event_t);
static osal_queue_t _usbd_q;

//--------------------------------------------------------------------+
// Prototypes
//--------------------------------------------------------------------+
static void mark_interface_endpoint(uint8_t ep2drv[8][2], uint8_t const* p_desc, uint16_t desc_len, uint8_t driver_id);
static bool process_control_request(uint8_t rhport, tusb_control_request_t* p_request);
static bool process_set_config(uint8_t rhport, uint8_t cfg_num);
static bool process_get_descriptor(uint8_t rhport, tusb_control_request_t const * p_request);

// from usbd_control.c
void usbd_control_reset(void);
void usbd_control_set_request(tusb_control_request_t const *request);
void usbd_control_set_complete_callback( bool (*fp) (uint8_t, tusb_control_request_t const * ) );
bool usbd_control_xfer_cb (uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);

//--------------------------------------------------------------------+
// Debug
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= 2
static char const* const _usbd_event_str[DCD_EVENT_COUNT] =
{
  "Invalid"        ,
  "Bus Reset"      ,
  "Unplugged"      ,
  "SOF"            ,
  "Suspend"        ,
  "Resume"         ,
  "Setup Received" ,
  "Xfer Complete"  ,
  "Func Call"
};

static char const* const _tusb_std_request_str[] =
{
  "Get Status"        ,
  "Clear Feature"     ,
  "Reserved"          ,
  "Set Feature"       ,
  "Reserved"          ,
  "Set Address"       ,
  "Get Descriptor"    ,
  "Set Descriptor"    ,
  "Get Configuration" ,
  "Set Configuration" ,
  "Get Interface"     ,
  "Set Interface"     ,
  "Synch Frame"
};

// for usbd_control to print the name of control complete driver
void usbd_driver_print_control_complete_name(bool (*control_complete) (uint8_t, tusb_control_request_t const * ))
{
  for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++)
  {
    usbd_class_driver_t const * driver = get_driver(i);
    if ( driver->control_complete == control_complete )
    {
      TU_LOG2("  %s control complete\r\n", driver->name);
      return;
    }
  }
}

#endif

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+
tusb_speed_t tud_speed_get(void)
{
  return (tusb_speed_t) _usbd_dev.speed;
}

bool tud_mounted(void)
{
  return _usbd_dev.configured;
}

bool tud_suspended(void)
{
  return _usbd_dev.suspended;
}

bool tud_remote_wakeup(void)
{
  // only wake up host if this feature is supported and enabled and we are suspended
  TU_VERIFY (_usbd_dev.suspended && _usbd_dev.remote_wakeup_support && _usbd_dev.remote_wakeup_en );
  dcd_remote_wakeup(TUD_OPT_RHPORT);
  return true;
}

bool tud_disconnect(void)
{
  TU_VERIFY(dcd_disconnect);
  dcd_disconnect(TUD_OPT_RHPORT);
  return true;
}

bool tud_connect(void)
{
  TU_VERIFY(dcd_connect);
  dcd_connect(TUD_OPT_RHPORT);
  return true;
}

//--------------------------------------------------------------------+
// USBD Task
//--------------------------------------------------------------------+
bool tud_init (void)
{
  TU_LOG2("USBD init\r\n");
  tu_varclr(&_usbd_dev);
	
#if CFG_TUSB_OS != OPT_OS_NONE
  // Init device mutex
	_usbd_mutex[0] = osal_mutex_create(&_ubsd_mutexdef[0]);
	_usbd_mutex[1] = osal_mutex_create(&_ubsd_mutexdef[1]);		
	TU_ASSERT(_usbd_mutex[0]);
	TU_ASSERT(_usbd_mutex[1]);

#endif	
	
  // Init device queue & task
  _usbd_q = osal_queue_create(&_usbd_qdef);
  TU_ASSERT(_usbd_q);
  // Get application driver if available
  if ( usbd_app_driver_get_cb )
  {
    _app_driver = usbd_app_driver_get_cb(&_app_driver_count);
  }
  // Init class drivers
  for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++)
  {
    usbd_class_driver_t const * driver = get_driver(i);
    TU_LOG2("%s init\r\n", driver->name);
    driver->init();
  }
  // Init device controller driver
  dcd_init(TUD_OPT_RHPORT);
  dcd_int_enable(TUD_OPT_RHPORT);
  return true;
}

void usbd_reset(uint8_t rhport)
{
  tu_varclr(&_usbd_dev);

  memset(_usbd_dev.itf2drv, DRVID_INVALID, sizeof(_usbd_dev.itf2drv)); // invalid mapping
  memset(_usbd_dev.ep2drv , DRVID_INVALID, sizeof(_usbd_dev.ep2drv )); // invalid mapping

  usbd_control_reset();

  for ( uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++ )
  {
    get_driver(i)->reset(rhport);
  }
}

bool tud_task_event_ready(void)
{
  // Skip if stack is not initialized
  if ( !tusb_inited() ) return false;
  return !osal_queue_empty(_usbd_q);
}


void tud_task (void)
{
  // Skip if stack is not initialized
  if ( !tusb_inited() ) return;
  // Loop until there is no more events in the queue
  while (1)
  {
    dcd_event_t event;
    if ( !osal_queue_receive(_usbd_q, &event) ) return;

#if CFG_TUSB_DEBUG >= 2
    if (event.event_id == DCD_EVENT_SETUP_RECEIVED) TU_LOG2("\r\n"); // extra line for setup
    TU_LOG2("USBD %s ", event.event_id < DCD_EVENT_COUNT ? _usbd_event_str[event.event_id] : "CORRUPTED");
#endif
		xSemaphoreTakeRecursive( USB_Semphr, portMAX_DELAY );
    switch ( event.event_id )
    {
      case DCD_EVENT_BUS_RESET:{
        TU_LOG2("\r\n");
        usbd_reset(event.rhport);
        _usbd_dev.speed = event.bus_reset.speed;
      break;
      }
      case DCD_EVENT_UNPLUGGED:{
        TU_LOG2("\r\n");
        usbd_reset(event.rhport);

        // invoke callback
        if (tud_umount_cb) tud_umount_cb(event.rhport);
      break;
			}
      case DCD_EVENT_SETUP_RECEIVED:{
        TU_LOG2_VAR(&event.setup_received);
        TU_LOG2("\r\n");

        // Mark as connected after receiving 1st setup packet.
        // But it is easier to set it every time instead of wasting time to check then set
        _usbd_dev.connected = 1;

        // Process control request
        if ( !process_control_request(event.rhport, &event.setup_received) )
        {
          TU_LOG2("  Stall EP0\r\n");
          // Failed -> stall both control endpoint IN and OUT
          dcd_edpt_stall(event.rhport, 0);
          dcd_edpt_stall(event.rhport, 0 | TUSB_DIR_IN_MASK);
        }
      break;
			}
      case DCD_EVENT_XFER_COMPLETE:{      
				 uint8_t ep_addr = event.xfer_complete.ep_addr;
				 uint8_t epnum   = tu_edpt_number(ep_addr);
				 uint8_t ep_dir  = tu_edpt_dir(ep_addr);
         TU_LOG2("on EP %02X with %u bytes\r\n", ep_addr, (unsigned int) event.xfer_complete.len);
        _usbd_dev.ep_status[epnum][ep_dir].busy  = false;
				_usbd_dev.ep_status[epnum][ep_dir].claimed = 0;
        if ( 0 == epnum )
        {
          usbd_control_xfer_cb(event.rhport, ep_addr, (xfer_result_t)event.xfer_complete.result, event.xfer_complete.len);
        }
        else
        {					
          usbd_class_driver_t const * driver = get_driver( _usbd_dev.ep2drv[epnum][ep_dir] );
          TU_LOG2("  %s xfer callback\r\n", driver->name);
          if(driver)
						driver->xfer_cb(event.rhport, ep_addr, event.xfer_complete.result, event.xfer_complete.len);
        }
				break;
		  }

      case DCD_EVENT_SUSPEND:{
        TU_LOG2("\r\n");
				usbd_reset(event.rhport);
        tud_suspend_cb(event.rhport); 
      break;
      }
      case DCD_EVENT_RESUME:{
        TU_LOG2("\r\n");
        if (tud_resume_cb) tud_resume_cb();
      break;
			}
      case DCD_EVENT_SOF:{
        TU_LOG2("\r\n");
        for ( uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++ )
        {
          usbd_class_driver_t const * driver = get_driver(i);
          if ( driver->sof ) driver->sof(event.rhport);
        }
      break;
			}
      case USBD_EVENT_FUNC_CALL:{
        TU_LOG2("\r\n");
        if ( event.func_call.func ) event.func_call.func(event.func_call.param);
      break;
      }
      default:
        TU_BREAKPOINT();
      break;
    }
		xSemaphoreGiveRecursive( USB_Semphr );
  }
}

//--------------------------------------------------------------------+
// Control Request Parser & Handling
//--------------------------------------------------------------------+

// Helper to invoke class driver control request handler
static bool invoke_class_control(uint8_t rhport, usbd_class_driver_t const * driver, tusb_control_request_t* request)
{
  usbd_control_set_complete_callback(driver->control_complete);
  TU_LOG2("  %s control request\r\n", driver->name);
  return driver->control_request(rhport, request);
}

// This handles the actual request and its response.
// return false will cause its caller to stall control endpoint
static bool process_control_request(uint8_t rhport, tusb_control_request_t* p_request)
{
  usbd_control_set_complete_callback(NULL);

  TU_ASSERT(p_request->bmRequestType_bit.type < TUSB_REQ_TYPE_INVALID);

  // Vendor request
  if ( p_request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR )
  {
    TU_VERIFY(tud_vendor_control_request_cb);

    if (tud_vendor_control_complete_cb) usbd_control_set_complete_callback(tud_vendor_control_complete_cb);
    return tud_vendor_control_request_cb(rhport, p_request);
  }

#if CFG_TUSB_DEBUG >= 2
  if (TUSB_REQ_TYPE_STANDARD == p_request->bmRequestType_bit.type && p_request->bRequest <= TUSB_REQ_SYNCH_FRAME)
  {
    TU_LOG2("  %s", _tusb_std_request_str[p_request->bRequest]);
    if (TUSB_REQ_GET_DESCRIPTOR != p_request->bRequest) TU_LOG2("\r\n");
  }
#endif

  switch ( p_request->bmRequestType_bit.recipient )
  {
    //------------- Device Requests e.g in enumeration -------------//
    case TUSB_REQ_RCPT_DEVICE:
      if ( TUSB_REQ_TYPE_CLASS == p_request->bmRequestType_bit.type )
      {
        uint8_t const itf = tu_u16_low(p_request->wIndex);
        TU_VERIFY(itf < TU_ARRAY_SIZE(_usbd_dev.itf2drv));

        usbd_class_driver_t const * driver = get_driver(_usbd_dev.itf2drv[itf]);
        TU_VERIFY(driver);

        // forward to class driver: "non-STD request to Interface"
        return invoke_class_control(rhport, driver, p_request);
      }
      if ( TUSB_REQ_TYPE_STANDARD != p_request->bmRequestType_bit.type )
      {
        // Non standard request is not supported
        TU_BREAKPOINT();
        return false;
      }

      switch ( p_request->bRequest )
      {
        case TUSB_REQ_SET_ADDRESS:
          // Depending on mcu, status phase could be sent either before or after changing device address,
          // or even require stack to not response with status at all
          // Therefore DCD must take full responsibility to response and include zlp status packet if needed.
          usbd_control_set_request(p_request); // set request since DCD has no access to tud_control_status() API
          dcd_set_address(rhport, (uint8_t) p_request->wValue);
          // skip tud_control_status()
          _usbd_dev.addressed = 1;
        break;

        case TUSB_REQ_GET_CONFIGURATION:
        {
          uint8_t cfgnum = _usbd_dev.configured ? 1 : 0;
          tud_control_xfer(rhport, p_request, &cfgnum, 1);
        }
        break;

        case TUSB_REQ_SET_CONFIGURATION:
        {
          uint8_t const cfg_num = (uint8_t) p_request->wValue;

          if ( !_usbd_dev.configured && cfg_num ) TU_ASSERT( process_set_config(rhport, cfg_num) );
          _usbd_dev.configured = cfg_num ? 1 : 0;

          tud_control_status(rhport, p_request);
        }
        break;

        case TUSB_REQ_GET_DESCRIPTOR:
          TU_VERIFY( process_get_descriptor(rhport, p_request) );
        break;

        case TUSB_REQ_SET_FEATURE:
          // Only support remote wakeup for device feature
          TU_VERIFY(TUSB_REQ_FEATURE_REMOTE_WAKEUP == p_request->wValue);

          // Host may enable remote wake up before suspending especially HID device
          _usbd_dev.remote_wakeup_en = true;
          tud_control_status(rhport, p_request);
        break;

        case TUSB_REQ_CLEAR_FEATURE:
          // Only support remote wakeup for device feature
          TU_VERIFY(TUSB_REQ_FEATURE_REMOTE_WAKEUP == p_request->wValue);

          // Host may disable remote wake up after resuming
          _usbd_dev.remote_wakeup_en = false;
          tud_control_status(rhport, p_request);
        break;

        case TUSB_REQ_GET_STATUS:
        {
          // Device status bit mask
          // - Bit 0: Self Powered
          // - Bit 1: Remote Wakeup enabled
          uint16_t status = (_usbd_dev.self_powered ? 1 : 0) | (_usbd_dev.remote_wakeup_en ? 2 : 0);
          tud_control_xfer(rhport, p_request, &status, 2);
        }
        break;

        // Unknown/Unsupported request
        default: TU_BREAKPOINT(); return false;
      }
    break;

    //------------- Class/Interface Specific Request -------------//
    case TUSB_REQ_RCPT_INTERFACE:
    {
      uint8_t const itf = tu_u16_low(p_request->wIndex);
      TU_VERIFY(itf < TU_ARRAY_SIZE(_usbd_dev.itf2drv));

      usbd_class_driver_t const * driver = get_driver(_usbd_dev.itf2drv[itf]);
      TU_VERIFY(driver);

      // all requests to Interface (STD or Class) is forwarded to class driver.
      // notable requests are: GET HID REPORT DESCRIPTOR, SET_INTERFACE, GET_INTERFACE
      if ( !invoke_class_control(rhport, driver, p_request) )
      {
        // For GET_INTERFACE, it is mandatory to respond even if the class
        // driver doesn't use alternate settings.
        TU_VERIFY( TUSB_REQ_TYPE_STANDARD == p_request->bmRequestType_bit.type &&
                   TUSB_REQ_GET_INTERFACE == p_request->bRequest);

        uint8_t alternate = 0;
        tud_control_xfer(rhport, p_request, &alternate, 1);
      }
    }
    break;

    //------------- Endpoint Request -------------//
    case TUSB_REQ_RCPT_ENDPOINT:
    {
      uint8_t const ep_addr = tu_u16_low(p_request->wIndex);
      uint8_t const ep_num  = tu_edpt_number(ep_addr);
      uint8_t const ep_dir  = tu_edpt_dir(ep_addr);

      TU_ASSERT(ep_num < TU_ARRAY_SIZE(_usbd_dev.ep2drv) );

      bool ret = false;

      // Handle STD request to endpoint
      if ( TUSB_REQ_TYPE_STANDARD == p_request->bmRequestType_bit.type )
      {
        // force return true for standard request
        ret = true;

        switch ( p_request->bRequest )
        {
          case TUSB_REQ_GET_STATUS:
          {
            uint16_t status = usbd_edpt_stalled(rhport, ep_addr) ? 0x0001 : 0x0000;
            tud_control_xfer(rhport, p_request, &status, 2);
          }
          break;

          case TUSB_REQ_CLEAR_FEATURE:
            if ( TUSB_REQ_FEATURE_EDPT_HALT == p_request->wValue ) usbd_edpt_clear_stall(rhport, ep_addr);
            tud_control_status(rhport, p_request);
          break;

          case TUSB_REQ_SET_FEATURE:
            if ( TUSB_REQ_FEATURE_EDPT_HALT == p_request->wValue ) usbd_edpt_stall(rhport, ep_addr);
            tud_control_status(rhport, p_request);
          break;

          // Unknown/Unsupported request
          default: TU_BREAKPOINT(); return false;
        }
      }

      usbd_class_driver_t const * driver = get_driver(_usbd_dev.ep2drv[ep_num][ep_dir]);

      if (driver)
      {
        // Some classes such as USBTMC needs to clear/re-init its buffer when receiving CLEAR_FEATURE request
        // We will forward all request targeted endpoint to class drivers after
        // - For class-type requests: driver is fully responsible to reply to host
        // - For std-type requests  : driver init/re-init internal variable/buffer only, and
        //                            must not call tud_control_status(), driver's return value will have no effect.
        //                            EP state has already affected (stalled/cleared)
        if ( invoke_class_control(rhport, driver, p_request) ) ret = true;
      }

      if ( TUSB_REQ_TYPE_STANDARD == p_request->bmRequestType_bit.type )
      {
        // Set complete callback = NULL since it can also stall the request.
        usbd_control_set_complete_callback(NULL);
      }

      return ret;
    }
    break;

    // Unknown recipient
    default: TU_BREAKPOINT(); return false;
  }

  return true;
}

// Process Set Configure Request
// This function parse configuration descriptor & open drivers accordingly
static bool process_set_config(uint8_t rhport, uint8_t cfg_num)
{
  tusb_desc_configuration_t const * desc_cfg = (tusb_desc_configuration_t const *) tud_descriptor_configuration_cb(cfg_num-1); // index is cfg_num-1
  TU_ASSERT(desc_cfg != NULL && desc_cfg->bDescriptorType == TUSB_DESC_CONFIGURATION);

  // Parse configuration descriptor
  _usbd_dev.remote_wakeup_support = (desc_cfg->bmAttributes & TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP) ? 1 : 0;
  _usbd_dev.self_powered = (desc_cfg->bmAttributes & TUSB_DESC_CONFIG_ATT_SELF_POWERED) ? 1 : 0;

  // Parse interface descriptor
  uint8_t const * p_desc   = ((uint8_t const*) desc_cfg) + sizeof(tusb_desc_configuration_t);
  uint8_t const * desc_end = ((uint8_t const*) desc_cfg) + desc_cfg->wTotalLength;

  while( p_desc < desc_end )
  {
    tusb_desc_interface_assoc_t const * desc_itf_assoc = NULL;

    // Class will always starts with Interface Association (if any) and then Interface descriptor
    if ( TUSB_DESC_INTERFACE_ASSOCIATION == tu_desc_type(p_desc) )
    {
      desc_itf_assoc = (tusb_desc_interface_assoc_t const *) p_desc;
      p_desc = tu_desc_next(p_desc); // next to Interface
    }

    TU_ASSERT( TUSB_DESC_INTERFACE == tu_desc_type(p_desc) );

    tusb_desc_interface_t const * desc_itf = (tusb_desc_interface_t const*) p_desc;
    uint16_t const remaining_len = desc_end-p_desc;

    uint8_t drv_id;
    for (drv_id = 0; drv_id < TOTAL_DRIVER_COUNT; drv_id++)
    {
      usbd_class_driver_t const *driver = get_driver(drv_id);
      uint16_t const drv_len = driver->open(rhport, desc_itf, remaining_len);

      if ( drv_len > 0 )
      {
        // Open successfully, check if length is correct
        TU_ASSERT( sizeof(tusb_desc_interface_t) <= drv_len && drv_len <= remaining_len);

        // Interface number must not be used already
        TU_ASSERT(DRVID_INVALID == _usbd_dev.itf2drv[desc_itf->bInterfaceNumber]);

        TU_LOG2("  %s opened\r\n", driver->name);
        _usbd_dev.itf2drv[desc_itf->bInterfaceNumber] = drv_id;

        // If IAD exist, assign all interfaces to the same driver
        if (desc_itf_assoc)
        {
          // IAD's first interface number and class should match with opened interface
          TU_ASSERT(desc_itf_assoc->bFirstInterface == desc_itf->bInterfaceNumber &&
                    desc_itf_assoc->bFunctionClass  == desc_itf->bInterfaceClass);

          for(uint8_t i=1; i<desc_itf_assoc->bInterfaceCount; i++)
          {
            _usbd_dev.itf2drv[desc_itf->bInterfaceNumber+i] = drv_id;
          }
        }

        mark_interface_endpoint(_usbd_dev.ep2drv, p_desc, drv_len, drv_id); // TODO refactor

        p_desc += drv_len; // next interface

        break;
      }
    }

    // Failed if cannot find supported driver
    TU_ASSERT(drv_id < TOTAL_DRIVER_COUNT);
  }

  // invoke callback
  if (tud_mount_cb) tud_mount_cb();

  return true;
}

// Helper marking endpoint of interface belongs to class driver
static void mark_interface_endpoint(uint8_t ep2drv[8][2], uint8_t const* p_desc, uint16_t desc_len, uint8_t driver_id)
{
  uint16_t len = 0;

  while( len < desc_len )
  {
    if ( TUSB_DESC_ENDPOINT == tu_desc_type(p_desc) )
    {
      uint8_t const ep_addr = ((tusb_desc_endpoint_t const*) p_desc)->bEndpointAddress;

      ep2drv[tu_edpt_number(ep_addr)][tu_edpt_dir(ep_addr)] = driver_id;
    }

    len   = (uint16_t)(len + tu_desc_len(p_desc));
    p_desc = tu_desc_next(p_desc);
  }
}

// return descriptor's buffer and update desc_len
static bool process_get_descriptor(uint8_t rhport, tusb_control_request_t const * p_request)
{
  tusb_desc_type_t const desc_type = (tusb_desc_type_t) tu_u16_high(p_request->wValue);
  uint8_t const desc_index = tu_u16_low( p_request->wValue );

  switch(desc_type)
  {
    case TUSB_DESC_DEVICE:
    {
      TU_LOG2(" Device\r\n");

      uint16_t len = sizeof(tusb_desc_device_t);

      // Only send up to EP0 Packet Size if not addressed
      // This only happens with the very first get device descriptor and EP0 size = 8 or 16.
      if ((CFG_TUD_ENDPOINT0_SIZE < sizeof(tusb_desc_device_t)) && !_usbd_dev.addressed)
      {
        len = CFG_TUD_ENDPOINT0_SIZE;

        // Hack here: we modify the request length to prevent usbd_control response with zlp
        ((tusb_control_request_t*) p_request)->wLength = CFG_TUD_ENDPOINT0_SIZE;
      }

      return tud_control_xfer(rhport, p_request, (void*) tud_descriptor_device_cb(), len);
    }
    break;

    case TUSB_DESC_BOS:
    {
      TU_LOG2(" BOS\r\n");

      // requested by host if USB > 2.0 ( i.e 2.1 or 3.x )
      if (!tud_descriptor_bos_cb) return false;

      tusb_desc_bos_t const* desc_bos = (tusb_desc_bos_t const*) tud_descriptor_bos_cb();

      uint16_t total_len;
      // Use offsetof to avoid pointer to the odd/misaligned address
      memcpy(&total_len, (uint8_t*) desc_bos + offsetof(tusb_desc_bos_t, wTotalLength), 2);

      return tud_control_xfer(rhport, p_request, (void*) desc_bos, total_len);
    }
    break;

    case TUSB_DESC_CONFIGURATION:
    {
      TU_LOG2(" Configuration[%u]\r\n", desc_index);

      tusb_desc_configuration_t const* desc_config = (tusb_desc_configuration_t const*) tud_descriptor_configuration_cb(desc_index);
      TU_ASSERT(desc_config);

      uint16_t total_len;
      // Use offsetof to avoid pointer to the odd/misaligned address
      memcpy(&total_len, (uint8_t*) desc_config + offsetof(tusb_desc_configuration_t, wTotalLength), 2);

      return tud_control_xfer(rhport, p_request, (void*) desc_config, total_len);
    }
    break;

    case TUSB_DESC_STRING:
      TU_LOG2(" String[%u]\r\n", desc_index);

      // String Descriptor always uses the desc set from user
      uint8_t const* desc_str = (uint8_t const*) tud_descriptor_string_cb(desc_index, p_request->wIndex);
      TU_VERIFY(desc_str);

      // first byte of descriptor is its size
      return tud_control_xfer(rhport, p_request, (void*) desc_str, desc_str[0]);
    break;

    case TUSB_DESC_DEVICE_QUALIFIER:
      TU_LOG2(" Device Qualifier\r\n");

      // Host sends this request to ask why our device with USB BCD from 2.0
      // but is running at Full/Low Speed. If not highspeed capable stall this request,
      // otherwise return the descriptor that could work in highspeed mode
      if ( tud_descriptor_device_qualifier_cb )
      {
        uint8_t const* desc_qualifier = tud_descriptor_device_qualifier_cb();
        TU_ASSERT(desc_qualifier);

        // first byte of descriptor is its size
        return tud_control_xfer(rhport, p_request, (void*) desc_qualifier, desc_qualifier[0]);
      }else
      {
        return false;
      }
    break;

    case TUSB_DESC_OTHER_SPEED_CONFIG:
      TU_LOG2(" Other Speed Configuration\r\n");

      // After Device Qualifier descriptor is received host will ask for this descriptor
      return false; // not supported
    break;

    default: return false;
  }
}

//--------------------------------------------------------------------+
// DCD Event Handler
//--------------------------------------------------------------------+
void dcd_event_handler(dcd_event_t const * event, bool in_isr)
{
  switch (event->event_id)
  {
    case DCD_EVENT_UNPLUGGED:
      _usbd_dev.connected  = 0;
      _usbd_dev.addressed  = 0;
      _usbd_dev.configured = 0;
      _usbd_dev.suspended  = 0;
      osal_queue_send(_usbd_q, event, in_isr);
    break;

    case DCD_EVENT_SOF:
      return;   // skip SOF event for now
    break;

    case DCD_EVENT_SUSPEND:
      // NOTE: When plugging/unplugging device, the D+/D- state are unstable and can accidentally meet the
      // SUSPEND condition ( Idle for 3ms ). Some MCUs such as SAMD doesn't distinguish suspend vs disconnect as well.
      // We will skip handling SUSPEND/RESUME event if not currently connected
//      if ( _usbd_dev.connected )
//      {			
        _usbd_dev.suspended = 1;
        osal_queue_send(_usbd_q, event, in_isr);
//      }
    break;

    case DCD_EVENT_RESUME:
      // skip event if not connected (especially required for SAMD)
      if ( _usbd_dev.connected )
      {
        _usbd_dev.suspended = 0;
        osal_queue_send(_usbd_q, event, in_isr);
      }
    break;

    default:
      osal_queue_send(_usbd_q, event, in_isr);
    break;
  }
}

void dcd_event_bus_signal (uint8_t rhport, dcd_eventid_t eid, bool in_isr)
{
  dcd_event_t event = { .rhport = rhport, .event_id = eid };
  dcd_event_handler(&event, in_isr);
}

void dcd_event_bus_reset (uint8_t rhport, tusb_speed_t speed, bool in_isr)
{
  dcd_event_t event = { .rhport = rhport, .event_id = DCD_EVENT_BUS_RESET };
  event.bus_reset.speed = speed;
  dcd_event_handler(&event, in_isr);
}

void dcd_event_setup_received(uint8_t rhport, uint8_t const * setup, bool in_isr)
{
  dcd_event_t event = { .rhport = rhport, .event_id = DCD_EVENT_SETUP_RECEIVED };
  memcpy(&event.setup_received, setup, 8);

  dcd_event_handler(&event, in_isr);
}

void dcd_event_xfer_complete (uint8_t rhport, uint8_t ep_addr, uint32_t xferred_bytes, uint8_t result, bool in_isr)
{
  dcd_event_t event = { .rhport = rhport, .event_id = DCD_EVENT_XFER_COMPLETE };

  event.xfer_complete.ep_addr = ep_addr;
  event.xfer_complete.len     = xferred_bytes;
  event.xfer_complete.result  = result;

  dcd_event_handler(&event, in_isr);
}

//--------------------------------------------------------------------+
// Helper
//--------------------------------------------------------------------+

// Parse consecutive endpoint descriptors (IN & OUT)
bool usbd_open_edpt_pair(uint8_t rhport, uint8_t const* p_desc, uint8_t ep_count, uint8_t xfer_type, uint8_t* ep_out, uint8_t* ep_in)
{
  for(int i=0; i<ep_count; i++)
  {
    tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) p_desc;

    TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType && xfer_type == desc_ep->bmAttributes.xfer);
    TU_ASSERT(usbd_edpt_open(rhport, desc_ep));

    if ( tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN )
    {
      (*ep_in) = desc_ep->bEndpointAddress;
    }else
    {
      (*ep_out) = desc_ep->bEndpointAddress;
    }

    p_desc = tu_desc_next(p_desc);
  }

  return true;
}

// Helper to defer an isr function
void usbd_defer_func(osal_task_func_t func, void* param, bool in_isr)
{
  dcd_event_t event =
  {
      .rhport   = 0,
      .event_id = USBD_EVENT_FUNC_CALL,
  };

  event.func_call.func  = func;
  event.func_call.param = param;

  dcd_event_handler(&event, in_isr);
}

//--------------------------------------------------------------------+
// USBD Endpoint API
//--------------------------------------------------------------------+

bool usbd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * desc_ep)
{
  TU_LOG2("  Open EP %02X with Size = %u\r\n", desc_ep->bEndpointAddress, desc_ep->wMaxPacketSize.size);

  return dcd_edpt_open(rhport, desc_ep);
}

bool usbd_edpt_claim(uint8_t cdc_msc, uint8_t ep_addr)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

#if CFG_TUSB_OS != OPT_OS_NONE
  osal_mutex_lock(_usbd_mutex[cdc_msc-1], OSAL_TIMEOUT_WAIT_FOREVER);
#endif

  // can only claim the endpoint if it is not busy and not claimed yet.
  bool ret = (_usbd_dev.ep_status[epnum][dir].busy == false) && (_usbd_dev.ep_status[epnum][dir].claimed == false);
  if (ret)
  {
    _usbd_dev.ep_status[epnum][dir].claimed = true;
  }

#if CFG_TUSB_OS != OPT_OS_NONE
  osal_mutex_unlock(_usbd_mutex[cdc_msc-1]);
#endif

  return ret;
}

bool usbd_edpt_release(uint8_t cdc_msc, uint8_t ep_addr)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

#if CFG_TUSB_OS != OPT_OS_NONE
  osal_mutex_lock(_usbd_mutex[cdc_msc-1], OSAL_TIMEOUT_WAIT_FOREVER);
#endif

  // can only release the endpoint if it is claimed and not busy
  bool ret = (_usbd_dev.ep_status[epnum][dir].busy == false) && (_usbd_dev.ep_status[epnum][dir].claimed == true);
  if (ret)
  {
    _usbd_dev.ep_status[epnum][dir].claimed = false;
  }

#if CFG_TUSB_OS != OPT_OS_NONE
  osal_mutex_unlock(_usbd_mutex[cdc_msc-1]);
#endif

  return ret;
}

bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  TU_LOG2("  Queue EP %02X with %u bytes ... ", ep_addr, total_bytes);

	// Attempt to transfer on busy or not claimed (skip now) endpoint, sound like an race condition !
  //TU_ASSERT(_usbd_dev.ep_status[epnum][dir].busy == 0 /*&& _usbd_dev.ep_status[epnum][dir].claimed == 1*/);
  while(_usbd_dev.ep_status[epnum][dir].busy == true)
	{
	   return false;
	}
	// Set busy first since the actual transfer can be complete before dcd_edpt_xfer() could return
  // and usbd task can preempt and clear the busy
  _usbd_dev.ep_status[epnum][dir].busy = true; 
	
  if ( dcd_edpt_xfer(rhport, ep_addr, buffer, total_bytes) )
  {
    TU_LOG2("OK\r\n");
    return true;
  }else
  {
    _usbd_dev.ep_status[epnum][dir].busy = false;
    TU_LOG2("failed\r\n");
    TU_BREAKPOINT();
    return false;
  }
}

bool usbd_edpt_busy(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  return _usbd_dev.ep_status[epnum][dir].busy;
}

void usbd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  dcd_edpt_stall(rhport, ep_addr);
  _usbd_dev.ep_status[epnum][dir].stalled = true;
  _usbd_dev.ep_status[epnum][dir].busy = true;
}

void usbd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  dcd_edpt_clear_stall(rhport, ep_addr);
  _usbd_dev.ep_status[epnum][dir].stalled = false;
  _usbd_dev.ep_status[epnum][dir].busy = false;
}

bool usbd_edpt_stalled(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);

  return _usbd_dev.ep_status[epnum][dir].stalled;
}

/**
 * usbd_edpt_close will disable an endpoint.
 * 
 * In progress transfers on this EP may be delivered after this call.
 * 
 */
void usbd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  TU_ASSERT(dcd_edpt_close, /**/);
  TU_LOG2("  CLOSING Endpoint: 0x%02X\r\n", ep_addr);

  dcd_edpt_close(rhport, ep_addr);

  return;
}

#endif