// QT Library
#include <QCoreApplication>
#include <QTime>


// Kinect2 Library
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>


// PCL Library
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/keypoints/brisk_2d.h>
#include <pcl/features/brisk_2d.h>


#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>

#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>

#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <TransformationEstimationTranslationOnly.h> // My Transformation Estimation

#include <pcl/registration/icp.h> //RegistrationICP
#include <pcl/registration/icp_nl.h> //RegistrationICP
#include <pcl/registration/gicp.h> //RegistrationICP
#include <pcl/registration/incremental_icp.h>




#include <pcl/filters/filter.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointWithScale PointBRISK;
typedef pcl::PointCloud<PointBRISK> PointCloudBRISK;
typedef pcl::BRISKSignature512 FeatureBRISK;
typedef pcl::PointCloud<FeatureBRISK> FeatureCloudBRISK;


// Adafruit Driver
#include <adafruit_uart.h>

#include <omp.h>

uint XStep = 150;
uint YStep = 70;
uint Size = 30;


using namespace std;


/*
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

    if (event.keyDown())
    {
        cout << "KeyboardEvent: " << event.getKeySym ()  << endl;

               if (event.getKeySym () == "h" )
        {
            cout << "HELP - Press:" << endl;
            cout << "   - o for Opening Kinect Connection" << endl;
            cout << "   - c for Closing Kinect Connection" << endl;
            cout << "   - p for Play/Pause Kinect Connection" << endl;
            cout << "   - h for Help" << endl;
        }

        if (event.getKeySym () == "o" )
            kin.Open()  ;

        if (event.getKeySym () == "c" )
            kin.Close();

        if (event.getKeySym () == "p" )
            play = !play;



    }
    return;
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton && event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;


        if (event.getY() < 80)
        {
            if (event.getX() < XStep*1)
            {
                if (!kin1.isOpen())
                {
                    if (kin1.Open(0) == SUCCESS)
                        viewer->updateText("CONN.", XStep*0, YStep*0+10, Size, 1, 1, 1, "CONNECT1");
                }
                else
                {
                    if (kin1.Close() == SUCCESS)
                        viewer->updateText("CONN.",  XStep*0, YStep*0+10, Size, 0.3,0.3,0.3, "CONNECT1");
                }
            }


            else if (event.getX() < XStep*2)
            {
                kin1._play= !kin1._play;
                if (kin1._play)
                    viewer->updateText("PLAY",  XStep*1, YStep*0+10, Size, 1, 1, 1, "PLAY1");

                else
                    viewer->updateText("PLAY",   XStep*1, YStep*0+10, Size, 0.3,0.3,0.3, "PLAY1");
            }


            else if (event.getX() < XStep*3)
            {
                kin1._save = !kin1._save;
                if (kin1._save)
                    viewer->updateText("SAVE",  XStep*2, YStep*0+10, Size, 1, 1, 1, "SAVE1");

                else
                    viewer->updateText("SAVE",  XStep*2, YStep*0+10, Size, 0.3, 0.3, 0.3, "SAVE1");
            }


        }

    }
}
*/


int main(int argc, char *argv[])
{


    //Adafruit
    QCoreApplication a(argc, argv);
    Adafruit_UART ada;
    if (ada.Open() != SUCCESS)
    {
        std::cout << "Error opening Adafruit Device!" << std::endl;
        return -1;
    }
    if (ada.Init() != SUCCESS)
    {
        std::cout << "Error initializing Adafruit Device!" << std::endl;
        return -1;
    }




    // Kinect2
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no Kinect2 connected!" << std::endl;
        return -1;
    }
    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
    std::string serial = freenect2.getDefaultDeviceSerialNumber();







    // Loop for parameters
    for(int argI = 1; argI < argc; ++argI)
    {
        const std::string arg(argv[argI]);

        // Pipeline
        if(arg == "cpu")        pipeline = new libfreenect2::CpuPacketPipeline();
        else if(arg == "gl")    pipeline = new libfreenect2::OpenGLPacketPipeline();
        else if(arg == "cl")    pipeline = new libfreenect2::OpenCLPacketPipeline();

        //check if parameter could be a serial number
        else if(arg.find_first_not_of("0123456789") == std::string::npos)       serial = arg;

        // Others
        else     std::cout << "Unknown argument: " << arg << std::endl;

    }






    // Initialize Kinect 2
    if(pipeline)    dev = freenect2.openDevice(serial, pipeline);
    else            dev = freenect2.openDevice(serial);

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());







    // Setup visualization
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    viewer->addCoordinateSystem (1.000);
    viewer->setCameraClipDistances(-10.000,10.000);
    viewer->setCameraPosition(0,0,4.000, // From where I am looking at
                              0,0,0, // Where I am looking at
                              1,0,0);   // What is the up orientation


    /*
    viewer->addText("CONN.",  XStep*0, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "CONNECT1");
    viewer->addText("PLAY",   XStep*1, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "PLAY1");
    viewer->addText("SAVE",   XStep*2, YStep*0 + 10, Size, 0.3, 0.3, 0.3, "SAVE1");
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

*/


    viewer->addText("FPS", XStep, 0, 10, 1, 1, 1, "FPS");
    Eigen::Matrix4f _PoseAdaOnKin;
    //_PoseAdaOnKin << -1, 0, 0, 0,   0, 0, 1, 0,     0, 1, 0, 0,   0, 0, 0, 1; // Top of the Kinect
    _PoseAdaOnKin << 1, 0, 0, 0,   0, -1, 0, 0,     0, 0, -1, 0,   0, 0, 0, 1; // back of the Kinect


    // BRISK constructor
    pcl::BriskKeypoint2D<PointT> brisk_keypoint_estimation;
    brisk_keypoint_estimation.setThreshold(60);
    brisk_keypoint_estimation.setOctaves(4);
    pcl::registration::CorrespondenceEstimation<PointT, PointT> correspondence_estimation_point;
    //  pcl::BRISK2DEstimation<PointT> brisk_descriptor_estimation;
    //   pcl::registration::CorrespondenceEstimation<FeatureBRISK, FeatureBRISK> correspondence_estimation_features;
    // std::vector<int> ind;


    QList<PointCloudT::Ptr> List_pointcloud;
    QList<PointCloudBRISK::Ptr> List_keypoint;
    QList<PointCloudT::Ptr> List_keypointcloud;
    QList<FeatureCloudBRISK::Ptr> List_descriptorcloud;


    QTime Timer;
    Timer.start();
    int i_frame = 0;
    while (!viewer->wasStopped ())
    {
        Timer.restart();


        // Handle Frames
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered);

        const float *undistorted_data = (float*)undistorted.data;
        const unsigned int *registered_data = (unsigned int*)registered.data;


        int width = undistorted.width;
        int height = undistorted.height;



        // Initialize my Point Cloud
        PointCloudT::Ptr PointCloud(new PointCloudT());
        PointCloud->height = 424;        // set the height
        PointCloud->width = 512;          // set the width
        PointCloud->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds
        PointCloud->resize(512 * 424); // set the memory size to allocate
        //PointCloud->sensor_origin_ = _KinectPose.block<4,1>(0,3);                               // set the translation of the Kinect
        //PointCloud->sensor_orientation_ = Eigen::Quaternionf(_KinectPose.block<3,3>(0,0));      // Set the rotation of the Kinect
        //PointCloud->header.stamp = timestamp.toMSecsSinceEpoch();                               // the stamp correspond to the acquisition time
        i_frame++;
        if (i_frame > 5)
            i_frame = 0;
        PointCloud->header.frame_id = QString("Kinect%1.pcd").arg(i_frame).toStdString();
        Eigen::Quaternionf myQuat;
        ada.GetQuat(&myQuat);
        PointCloud->sensor_orientation_ = myQuat * Eigen::Quaternionf(_PoseAdaOnKin.block<3,3>(0,0));      // Set the rotation of the Kinect







        // Set data into my Point cloud
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

        int good = 0;
#pragma omp parallel for
        for (unsigned int i = 0; i < height ;i++)
        {
            float k_y = (dev->getIrCameraParams().cy - i) / dev->getIrCameraParams().fy;

            for (unsigned int j = 0; j < width ;j++)
            {
                float k_x = (dev->getIrCameraParams().cx - j) / dev->getIrCameraParams().fx;


                // Measurement are in millimeters -> Not Projectable!!
                // float depth = undistorted_data[i * width + j] ;
                // Measurement are in meter -> Now it is Projectable!!
                float depth = undistorted_data[i * width + j] / 1000.0f;

                unsigned int rgba = registered_data[i * width + j];

                PointT P;

                if ( depth > 0.001 && rgba != 0)
                {
                    P.x = -depth * k_x;
                    P.y =  depth * k_y;
                    P.z =  depth;

                    P.a = (rgba >> 24) & 0xFF;
                    P.r = (rgba >> 16) & 0xFF;
                    P.g = (rgba >> 8)  & 0xFF;
                    P.b =  rgba        & 0xFF;
                    good++;
                }
                else
                {
                    P.x = P.y = P.z = badPoint;
                    P.rgba = 0;
                }

                PointCloud->at(j,i) = P;
            }
        }






        // Get BRISK


        //output
        PointCloudBRISK::Ptr brisk_keypoints_2D (new PointCloudBRISK());
        brisk_keypoint_estimation.setInputCloud (PointCloud);
        brisk_keypoint_estimation.compute (*brisk_keypoints_2D);


        //convert pointwithscale to 3D
        PointCloudT::Ptr brisk_keypoints_3D (new PointCloudT());
        brisk_keypoints_3D->resize(brisk_keypoints_2D->size());

        int k = brisk_keypoints_2D->size();
        for(int i = 0, j = 0; i < k; ++i)
        {
            /// TO DO: improve accuracy
            int u = floor(brisk_keypoints_2D->points[i].x + 0.5);
            int v = floor(brisk_keypoints_2D->points[i].y + 0.5);

            j = u + v * PointCloud->width;

            if(isnan(PointCloud->points[j].x))
            {
                --k;
            }
            else
            {
                brisk_keypoints_3D->points[i].b = PointCloud->points[j].b;
                brisk_keypoints_3D->points[i].g = PointCloud->points[j].g;
                brisk_keypoints_3D->points[i].r = PointCloud->points[j].r;
                brisk_keypoints_3D->points[i].x = PointCloud->points[j].x;
                brisk_keypoints_3D->points[i].y = PointCloud->points[j].y;
                brisk_keypoints_3D->points[i].z = PointCloud->points[j].z;
            }
        }

        std::vector<PointT,Eigen::aligned_allocator<PointT> >::iterator  keypointIt=brisk_keypoints_3D->begin();

        for(size_t i=k; k<brisk_keypoints_2D->size(); ++k)
            brisk_keypoints_3D->erase(keypointIt+i);





        if (List_keypointcloud.size() > 0)
        {
            Eigen::Matrix4f PC_Pose = Eigen::Matrix4f::Identity();
            PC_Pose.block(0,0,3,3) = PointCloud->sensor_orientation_.matrix();
            PC_Pose.block(0,3,3,1) = PointCloud->sensor_origin_.head(3);


            Eigen::Matrix4f inv_old_PC_Pose = Eigen::Matrix4f::Identity();
            inv_old_PC_Pose.block(0,0,3,3) = List_pointcloud.back()->sensor_orientation_.matrix().transpose();
            inv_old_PC_Pose.block(0,3,3,1) = -inv_old_PC_Pose.block<3,3>(0,0) * List_pointcloud.back()->sensor_origin_.head(3);

            PointCloudT::Ptr old_brisk_keypoints_3D = List_keypointcloud.back();


            /*  REGISTRATION
            correspondence_estimation_point.setInputSource (brisk_keypoints_3D);
            correspondence_estimation_point.setInputTarget (old_brisk_keypoint_3D);

            pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
            correspondence_estimation_point.determineReciprocalCorrespondences (*correspondences);


            pcl::registration::TransformationEstimationSVD<PointT, PointT> te;

            te.estimateRigidTransformation(*brisk_keypoints_3D, *old_brisk_keypoint_3D, *correspondences, trans );

*/
            /* ICP */


            // ICP
            pcl::IterativeClosestPoint<PointT, PointT> icp;
            //CORRESPONDENCE ESTIMATION
            pcl::registration::CorrespondenceEstimation<PointT, PointT>::Ptr cens (new pcl::registration::CorrespondenceEstimation<PointT, PointT>);
            cens->setInputSource (brisk_keypoints_3D);
            cens->setInputTarget (old_brisk_keypoints_3D);
            icp.setCorrespondenceEstimation (cens);
            //CORRESPONDENCE REJECTION
            pcl::registration::CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new pcl::registration::CorrespondenceRejectorOneToOne);
            icp.addCorrespondenceRejector (cor_rej_o2o);
            icp.setMaxCorrespondenceDistance(0.1);
            // TRANSFORMATION ESTIMATION
            pcl::registration::TransformationEstimationTranslationOnly<PointT, PointT>::Ptr te (new pcl::registration::TransformationEstimationTranslationOnly<PointT, PointT>);
            icp.setTransformationEstimation (te);
            //ICP
            icp.setMaximumIterations(5);
            icp.setInputTarget(old_brisk_keypoints_3D);
            icp.setInputSource (brisk_keypoints_3D);
            icp.align(*brisk_keypoints_3D, inv_old_PC_Pose * PC_Pose);


            Eigen::Matrix4f trans = icp.getFinalTransformation ();

            std::cout << trans << std::endl;

            //    PointCloud->sensor_orientation_ = trans * PointCloud->sensor_orientation_;


            PC_Pose = trans * PC_Pose;

            PointCloud->sensor_origin_ = PC_Pose.block<4,1>(0,3);
            PointCloud->sensor_orientation_ = Eigen::Quaternionf(PC_Pose.block<3,3>(0,0));


            //  correspondence_estimation_features.determineCorrespondences (*correspondences);
        }


        /*
        std::cout << "size of keypoints : " << brisk_keypoints_2D->size();
        pcl::removeNaNFromPointCloud(*brisk_keypoints_2D, *brisk_keypoints_2D, ind);
        std::cout << "  -->>  reduced at : " << brisk_keypoints_2D->size() << std::endl;

        FeatureCloudBRISK::Ptr brisk_desciption_2D (new FeatureCloudBRISK());
        brisk_descriptor_estimation.setInputCloud (PointCloud);
        brisk_descriptor_estimation.setKeypoints (brisk_keypoints_2D);
        brisk_descriptor_estimation.compute (*brisk_desciption_2D);

        std::cout << "size of desccriptor : " << brisk_desciption_2D->size();
       // pcl::removeNaNFromPointCloud(*brisk_desciption_2D, *brisk_desciption_2D, ind);
        std::cout << "  -->>  reduced at : " << brisk_desciption_2D->size() << std::endl;
*/
        /*  if (List_descriptorcloud.size() > 0)
        {
            FeatureCloudBRISK::Ptr old_brisk_description_2D = List_descriptorcloud.back();

            correspondence_estimation_features.setInputSource (brisk_desciption_2D);
            correspondence_estimation_features.setInputTarget (old_brisk_description_2D);

            pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
            correspondence_estimation_features.determineReciprocalCorrespondences (*correspondences);
          //  correspondence_estimation_features.determineCorrespondences (*correspondences);
        }*/



        List_pointcloud.append(PointCloud);
        List_keypoint.append(brisk_keypoints_2D);
        List_keypointcloud.append(brisk_keypoints_3D);
        //   List_descriptorcloud.append(brisk_desciption_2D);





        std::cout << QDateTime::currentDateTime().toString().toStdString() << " time elapsed :" << Timer.elapsed() << "  PointCloud is : " << good << std::endl;



        viewer->updateText(QString("FPS = %1 Hz").arg(1000.0f/Timer.elapsed()).toStdString(),XStep,0,10,1,1,1,"FPS");

        pcl::visualization::PointCloudColorHandlerRGBField<PointT> KinectColor(PointCloud);
        if (viewer->contains(PointCloud->header.frame_id))
        {
            viewer->updatePointCloud(PointCloud,KinectColor,PointCloud->header.frame_id);
            Eigen::Matrix4f currentPose = Eigen::Matrix4f::Identity();
            currentPose.block(0,0,3,3) = PointCloud->sensor_orientation_.matrix();
            currentPose.block(0,3,3,1) = PointCloud->sensor_origin_.head(3);
            viewer->updatePointCloudPose(PointCloud->header.frame_id,Eigen::Affine3f(currentPose) );
        }
        else
            viewer->addPointCloud(PointCloud,KinectColor,PointCloud->header.frame_id);


        viewer->spinOnce (1);



        // cout << QString("Acquisition Time = %1 ms").arg(t.elapsed()).toStdString() << endl;
        //  viewer->updateText(QString("Acquisition Time = %1 ms").arg(t.elapsed()).toStdString(),50,50,20,1,1,1,"AcqTime");

    }






    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    ada.close();
    dev->stop();
    dev->close();


    return 0;
}
