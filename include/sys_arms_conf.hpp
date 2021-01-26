#ifndef SYS_ARMS_CONF_HPP
#define SYS_ARMS_CONF_HPP

namespace CONF {


typedef enum {
  ARMS_M_SUPR_ID = 0, /*0 is special one, don't change it */
  ARMS_M_1_ID = 1,
  ARMS_M_2_ID,
  ARMS_M_3_ID,
  ARMS_M_4_ID,
  ARMS_M_5_ID,
  ARMS_M_6_ID,
  ARMS_M_7_ID,
  ARMS_M_8_ID,
  ARMS_M_9_ID,
  ARMS_M_10_ID,
  ARMS_M_11_ID,
  ARMS_M_MAX_ID,
} DMS_MODULE_NAME_ID;


//FN=Feature Name
typedef enum {
  /* sdk report key event, dms support save this event or not */
  DMS_FN_KEY_EVENT_SAVE = 0,
  /* sdk report key event, dms support upload this event or not */
  DMS_FN_KEY_EVENT_UPLOAD = 1,
  /* dms get image from camera */
  DMS_FN_IMAGE_FROM_CAMERA = 2,
  /* dms save raw image data from camera */
  DMS_FN_IMAGE_RECORDER = 3,
  /* dms support ethernet diagnostic */
  DMS_FN_ETHERNET_DIAG = 4,
  /* dms using gray dcu output */
  DMS_FN_DCU_WITH_GRAY = 5,
  /* dms using color dcu output */
  DMS_FN_DCU_WITH_COLOR = 6,
  /*
   * dms may be support a53 send can message in feature, so, add this feature
   * to support it, icc send from a53 to m4, m4 send can message */
  DMS_FN_ICC_TO_M4 = 7,
  /*
   * dms may be support a53 send can message in feature, so, add this feature
   * icc send from a53 one module to other module on same a53, a53 send can
   * message */
  DMS_FN_ICC_TO_A53 = 8,
  /*using h264 and gstreamer for video cast*/
  DMS_FN_VIDEOCAST = 9,
  /* dms get image from local saved image, not from camera */
  DMS_FN_IMAGE_FROM_RAW_DATA = 10,
  /* dms get image from local saved video, not from camera */
  DMS_FN_IMAGE_FROM_VIDEO = 11,
  /* dms running chess board*/
  DMS_FN_CHESS_BOARD = 12,
  DMS_FN_MAX,
} DMS_FEATURE_NAME;


//MN=Module Name
//
/*
 * arms size
 */
#define DEF_SYS_ARMS_NUMS 11

const char MN_NAME[][15] = {"MN_SUPR", "MN_SERVER1", "MN_SERVER2",
                            "MN_SERVER3", "MN_SERVER4", "MN_SERVER5",
                            "MN_SERVER6", "MN_SERVER7", "MN_SERVER8",
                            "MN_SERVER9", "MN_SERVER10", "MN_SERVER11"};

/*
const char MN_SERVER_IP[][16] = {"127.0.0.0", "127.0.0.0", "127.0.0.0",
                                 "127.0.0.0", "127.0.0.0", "127.0.0.0",
                                 "127.0.0.0", "127.0.0.0", "127.0.0.0",
                                 "127.0.0.0", "127.0.0.0", "127.0.0.0" };
*/

const char MN_SERVER_IP[][16] = {"192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100" };

const int  MN_SERVER_PORT[DEF_SYS_ARMS_NUMS + 1] = { 8001, 8002, 8003,
                                                     8004, 8005, 8006,
                                                     8007, 8008, 8009,
                                                     8010, 8011, 8012};

const int SERVER_UDP_TIMEOUT = 5000;


//Define priority for modules
const int PRI_SUPR = 50;
const int PRI_LEAD = 70;


///for dms common part/////////////////////////////////////////////////////////
//Define the image size of camera
const int HI_CAM_WIDTH = 1280;
const int HI_CAM_HEIGHT = 720;
const int HI_SCALE_WIDTH = 640;
const int HI_SCALE_HEIGHT = 360;
///end of dms common part

///for video cast //////////////////////////////////////////////////////////////
const int RTMPSERVERADDRLENGTH = 50;
const int H264FBUFFERSIZE = 800000;
///end of video cast


///Path under "/hietc/dms" /////////////////////////////////////////////////////
#define DMS_ETC_PREFIX "/hietc/dms"
//Define mmt database path
const char HI_MMT_DB_NAME[] = DMS_ETC_PREFIX"/dms.cdb";

//Define leveldb database file
const char HI_FACEID_LEVELDB_FILE[] = DMS_ETC_PREFIX"/faceidNameLevelDB.db";

const char HI_PROGRAM_ENTER_FLAG[] = DMS_ETC_PREFIX"/PEF";
const char HI_SOFTWARE_VALID_FLAG[] = DMS_ETC_PREFIX"/SVF";


///Path under "/hiappact/dms" //////////////////////////////////////////////////
#define DMS_APP_PREFIX "/hiappact/dms"
const char DMS_COMMON_CONF[] = DMS_APP_PREFIX"/etc/hidms.ini";

const char DMS_MMT_MODEL_FILE[] = DMS_APP_PREFIX"/model/models.rd";
const char MMT_ICON_YAWN_POS[] = DMS_APP_PREFIX"/icon/yawn_pos.jpg";
const char MMT_ICON_YAWN_NEG[] = DMS_APP_PREFIX"/icon/yawn_neg.jpg";

const char MMT_ICON_DRINK_POS[] = DMS_APP_PREFIX"/icon/drink_pos.jpg";
const char MMT_ICON_DRINK_NEG[] = DMS_APP_PREFIX"/icon/drink_neg.jpg";

const char MMT_ICON_SMOKE_POS[] = DMS_APP_PREFIX"/icon/smoke_pos.jpg";
const char MMT_ICON_SMOKE_NEG[] = DMS_APP_PREFIX"/icon/smoke_neg.jpg";

const char MMT_ICON_EYS_CLOSE_POS[] = DMS_APP_PREFIX"/icon/eye_close_pos.jpg";
const char MMT_ICON_EYS_CLOSE_NEG[] = DMS_APP_PREFIX"/icon/eye_close_neg.jpg";

const char MMT_ICON_PHONE_CALL_POS[] = DMS_APP_PREFIX"/icon/phone_call_pos.jpg";
const char MMT_ICON_PHONE_CALL_NEG[] = DMS_APP_PREFIX"/icon/phone_call_neg.jpg";


///Path under "/hidata/dms" ////////////////////////////////////////////////////
#define DMS_DATA_PREFIX "/hidata/dms"
const char DMS_LOCAL_WRITE_RAW_DATA_PATH[] = DMS_DATA_PREFIX"/writerawdata";
const char DMS_LOCAL_READ_RAW_DATA_PATH[] = DMS_DATA_PREFIX"/readrawdata";
const char DMS_LOCAL_READ_VIDEO_FILE[] = DMS_DATA_PREFIX"/readvideo/hi_dms_video.avi";
//Define folder path for saving warning images
const char HI_SAVE_IMG_FOLDER[] = DMS_DATA_PREFIX"/warningImg/";


///Path under "/hilog/dms" /////////////////////////////////////////////////////
#define DMS_SYSLOG_PREFIX "/hilog"

///for diag ///////////////////////////////////////////////////////////////////
//Define driver path and buffer size for diag system info modules
const char HI_CHIP_TEM_DEV[] = "/sys/class/hwmon/hwmon0/temp1_input";
const int FILE_SYSTEM_REMAIN_PERCENT = 4; //4 means 1/4 = 0.25, file systems remain 25% at least.
const int FREE_RAM_REMAIN_PERCENT = 4; //4 means 1/4 = 0.25, free ram remains 25% at least.
const int BUFFLEGTH = 20;
const int LOWERTEMPLIMIT = -40;
const int UPPERTEMPLIMIT = 125;
const int HI_KERNEL_RELEASE[3] = {4, 14, 34};
const int HI_CPU_TEMPERATURE_HIGH_THRESHOLD = 100; //in Celsius
const int HI_CPU_TEMPERATURE_LOW_THRESHOLD = 90; //in Celsius
const int HI_CPU_HIGH_TEMPERATURE_LAST_TIME = 20 * 1000; //in ms
const int HI_CPU_LOW_TEMPERATURE_LAST_TIME = 10 * 1000; //in ms
const char HI_CPU_DIAG_DEV[] = "/proc/stat";
const char HI_MEM_DIAG_DEV[] = "/proc/meminfo";
const int HI_MAX_CPU_LOAD = 143; // = (cpuBusyTime + cpuIdelTime) * 100 / cpuIdelTime, 30%
const int HI_CAM_TEMPERATURE_HIGH_THRESHOLD = 105; //in Celsius
const int HI_CAM_TEMPERATURE_LOW_THRESHOLD = 100; //in Celsius
const int HI_CAM_HIGH_TEMPERATURE_LAST_TIME = 20 * 1000; //20s
const int HI_CAM_LOW_TEMPERATURE_LAST_TIME = 10 * 1000; //10s
///end of diag


const int DMS_MAX_READ_SAVE_IMAGE_CNT = 5000;
//const int DMS_MAX_READ_SAVE_IMAGE_CNT = 700;

//Define type for saving warning images
const char HI_SAVE_IMG_TYPE[] = ".jpg";
///end of save image

}  //namespace

#endif // SYS_ARMS_CONF_HPP
