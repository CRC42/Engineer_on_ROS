#include "CameraApi.h" //相机SDK的API头文件

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

class CameraNode
{
    protected:
        int                     iCameraCounts = 1;
        int                     iStatus=-1;
        tSdkCameraDevInfo       tCameraEnumList;
        int                     hCamera;
        tSdkCameraCapbility     tCapability;      // 设备描述信息
        tSdkFrameHead           sFrameInfo;
        BYTE*			        pbyBuffer;
        int                     channel=3;
        double                  exposeTime;       // 曝光时间
        unsigned char           * g_pRgbBuffer;     // 处理后数据缓存区
        int                     iCameraFPS;       // 最大帧率
        int                     iModeSel;         // 触发模式
        ros::NodeHandle nh;
        image_transport::Publisher pub;
    public:
        CameraNode();
        bool CameraNodeInit();
        bool CameraRunTask();
        bool CameraRunSoftTriggerTask(ros::Time stamp);
        void CameraPublish(ros::Time stamp = ros::Time::now());
        bool CameraRelease();
};

/** 
 * @brief 初始化相机参数
 * @return 相机初始化成功
 */
CameraNode::CameraNode()
{
    image_transport::ImageTransport it(nh);
    pub = it.advertise("/camera/color/raw_image", 1);
    nh.param<double>("camera/exposeTime", exposeTime, 4000);
    nh.param<int>("camera/FPS", iCameraFPS, 30);
    nh.param<int>("camera/iModeSel",iModeSel, 0);
};

/** 
 * @brief 相机初始化，一般都要调用
 */
bool CameraNode::CameraNodeInit()
{
    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
	printf("state = %d\n", iStatus);

	printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        perror("[MindVision] No Device Found");
        return false;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //初始化失败
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        perror("[MindVision] Camera Init failed");
        return false;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */

    // 帧率
    CameraSetFrameSpeed(hCamera, FRAME_SPEED_LOW);

    // 曝光
    CameraSetAeState(hCamera, FALSE);  // 设置手动曝光
    CameraSetExposureTime(hCamera,exposeTime);
    double tmp_exposeTime = 0;
    CameraGetExposureTime(hCamera, &tmp_exposeTime);
    printf("[MindVision] exposeTime = %f us\n", tmp_exposeTime);

    // 触发模式
    CameraSetTriggerMode(hCamera, iModeSel);
    INT tmp_piModeSel = 0;
    CameraGetTriggerMode(hCamera,&tmp_piModeSel);
    printf("[MindVision] TriggerMode = %d \n", tmp_piModeSel);
    CameraPlay(hCamera);
    
    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */

    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }
    return true;
};

/** 
 * @brief 相机在连续工作模式下，向外发布数据
 */
bool CameraNode::CameraRunTask()
{
    ros::Rate loop_rate(iCameraFPS);
    while(ros::ok())
    {
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
		{
		    CameraPublish();
		}
    }
    return true;
}

/** 
 * @brief 相机反初始化
 */
bool CameraNode::CameraRelease()
{
    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);
    return true;
}

/** 
 * @brief 相机发布数据到image_topic
 */
void CameraNode::CameraPublish(ros::Time stamp)
{
    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
        
        cv::Mat matImage(
                Size(sFrameInfo.iWidth,sFrameInfo.iHeight), 
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer
                );
        std_msgs::Header header;
        header.stamp = stamp;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", matImage).toImageMsg();
        pub.publish(msg);
        CameraReleaseImageBuffer(hCamera,pbyBuffer);
}

/** 
 * @brief 相机在软同步模式下，向外发布数据
 */
bool CameraNode::CameraRunSoftTriggerTask(ros::Time stamp)
{

    if(CameraSoftTrigger(hCamera)==CAMERA_STATUS_SUCCESS && CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
    {
        CameraPublish(stamp);
    }
    return true;
}