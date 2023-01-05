# include <CameraNode.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    CameraNode CN1;
    if(CN1.CameraNodeInit())
    {
        CN1.CameraRunTask();
        CN1.CameraRelease();
    }
    return 0;
}

