10. Tracking 跟踪
===========================

获取图像并进行跟踪

GramImage


::
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);


更改补偿信息

::

    void ChangeCalibration(const string &strSettingPath);

跟踪状态

- 系统未准备
- 还没有图像
- 没有初始化
- 跟踪OK
- 跟踪丢失

::
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;


输入传感器,当前帧

::

    // Input sensor:MONOCULAR, STEREO, RGBD
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

Initialization Variables (Monocular)
初始化时前两帧相关变量

::
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;// 跟踪初始化时前两帧之间的匹配
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

Lists used to recover the full camera trajectory at the end of the execution.
Basically we store the reference keyframe for each frame and its relative transformation

::
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;




其他函数
------------

::

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    // orb特征提取器，不管单目还是双目，mpORBextractorLeft都要用到
    // 如果是双目，则要用到mpORBextractorRight
    // 如果是单目，在初始化的时候使用mpIniORBextractor而不是mpORBextractorLeft，
    // mpIniORBextractor属性中提取的特征点个数是mpORBextractorLeft的两倍
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    // 单目初始器
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;// 当前关键帧就是参考帧
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
