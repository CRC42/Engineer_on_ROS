#include <CameraNode.h>
#include <ros/package.h>
#include <string.h>

class CameraSaveNode:public CameraNode
{
    public:
    int                     iDisplayFrames;
    std::string             packagePath;
    void saveFrame();
};

void CameraSaveNode::saveFrame()
{
    nh.param<int>("camera/iDisplayFrames", iDisplayFrames, 500);
    printf("iDisplayFrames = %d\n", iDisplayFrames);
    int numFrames = iDisplayFrames;
    packagePath= ros::package::getPath("sensor_ros_driver");
    printf("packagePath = %s\n", packagePath.c_str());
    while(iDisplayFrames--)
    {
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
		{
		    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
		    
		    cv::Mat matImage(
					Size(sFrameInfo.iWidth,sFrameInfo.iHeight), 
					sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					g_pRgbBuffer
					);
            std::string saveImage = packagePath + "/calibrationdata/" + std::to_string(numFrames-iDisplayFrames) + ".jpg";
            cv::imwrite(saveImage,matImage);
            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			//否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera,pbyBuffer);
		}
    }

    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_demo");
    CameraSaveNode CSN1;
    CSN1.CameraNodeInit();
    CSN1.saveFrame();

    return 0;
}
