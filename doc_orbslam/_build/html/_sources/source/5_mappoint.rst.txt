5.MapPoint 路标点,地图点
==================================

设置世界坐标,得到世界坐标

::

      void SetWorldPos(const cv::Mat &Pos);
      cv::Mat GetWorldPos();

归一化

::

      cv::Mat GetNormal();

得到参考的关键帧

::

      KeyFrame* GetReferenceKeyFrame();
  

观测点

::

      std::map<KeyFrame*,size_t> GetObservations();
      int Observations();
      void AddObservation(KeyFrame* pKF,size_t idx);
      void EraseObservation(KeyFrame* pKF);

关键帧的index

::
  
      int GetIndexInKeyFrame(KeyFrame* pKF);
      bool IsInKeyFrame(KeyFrame* pKF);
  
      void SetBadFlag();
      bool isBad();
  
      void Replace(MapPoint* pMP);
      MapPoint* GetReplaced();
  
      void IncreaseVisible(int n=1);
      void IncreaseFound(int n=1);
      float GetFoundRatio();
      inline int GetFound(){
          return mnFound;
      }


计算描述子

::
  
      void ComputeDistinctiveDescriptors();
  
      cv::Mat GetDescriptor();
  
      void UpdateNormalAndDepth();
  

计算最大，最小距离方差

::

      float GetMinDistanceInvariance();
      float GetMaxDistanceInvariance();
      int PredictScale(const float &currentDist, KeyFrame*pKF);
      int PredictScale(const float &currentDist, Frame* pF);


Tracking
---------------

TrackLocalMap - SearchByProjection中决定是否对该点进行投影的变量

mbTrackInView==false的点有几种：

- 已经和当前帧经过匹配（TrackReferenceKeyFrame，TrackWithMotionModel）但在优化过程中认为是外点

- 已经和当前帧经过匹配且为内点，这类点也不需要再进行投影

- 不在当前相机视野中的点（即未通过isInFrustum判断）


3D Descriptor
---------------

每个3D点也有一个descriptor

如果MapPoint与很多帧图像特征点对应（由keyframe来构造时），那么距离其它描述子的平均距离最小的描述子是最佳描述子

MapPoint只与一帧的图像特征点对应（由frame来构造时），那么这个特征点的描述子就是该3D点的描述子

::

  
  public:
      long unsigned int mnId; ///< Global ID for MapPoint
      static long unsigned int nNextId;
      const long int mnFirstKFid; ///< 创建该MapPoint的关键帧ID
      const long int mnFirstFrame; ///< 创建该MapPoint的帧ID（即每一关键帧有一个帧ID）
      int nObs;
  
      // Variables used by the tracking
      float mTrackProjX;
      float mTrackProjY;
      float mTrackProjXR;
      int mnTrackScaleLevel;
      float mTrackViewCos;

      // TrackLocalMap - SearchByProjection中决定是否对该点进行投影的变量
      // mbTrackInView==false的点有几种：
      // a 已经和当前帧经过匹配（TrackReferenceKeyFrame，TrackWithMotionModel）但在优化过程中认为是外点
      // b 已经和当前帧经过匹配且为内点，这类点也不需要再进行投影
      // c 不在当前相机视野中的点（即未通过isInFrustum判断）
      bool mbTrackInView;
      // TrackLocalMap - UpdateLocalPoints中防止将MapPoints重复添加至mvpLocalMapPoints的标记
      long unsigned int mnTrackReferenceForFrame;
      // TrackLocalMap - SearchLocalPoints中决定是否进行isInFrustum判断的变量
      // mnLastFrameSeen==mCurrentFrame.mnId的点有几种：
      // a 已经和当前帧经过匹配（TrackReferenceKeyFrame，TrackWithMotionModel）但在优化过程中认为是外点
      // b 已经和当前帧经过匹配且为内点，这类点也不需要再进行投影
      long unsigned int mnLastFrameSeen;
  
      // Variables used by local mapping
      long unsigned int mnBALocalForKF;
      long unsigned int mnFuseCandidateForKF;
  
      // Variables used by loop closing
      long unsigned int mnLoopPointForKF;
      long unsigned int mnCorrectedByKF;
      long unsigned int mnCorrectedReference;
      cv::Mat mPosGBA;
      long unsigned int mnBAGlobalForKF;
  
  
      static std::mutex mGlobalMutex;
  
  protected:
  
      // Position in absolute coordinates
      cv::Mat mWorldPos; ///< MapPoint在世界坐标系下的坐标
  
      // Keyframes observing the point and associated index in keyframe
      std::map<KeyFrame*,size_t> mObservations; ///< 观测到该MapPoint的KF和该MapPoint在KF中的索引
  
      // Mean viewing direction
      // 该MapPoint平均观测方向
      cv::Mat mNormalVector;
  
      // Best descriptor to fast matching
      // 每个3D点也有一个descriptor
      // 如果MapPoint与很多帧图像特征点对应（由keyframe来构造时），那么距离其它描述子的平均距离最小的描述子是最佳描述子
      // MapPoint只与一帧的图像特征点对应（由frame来构造时），那么这个特征点的描述子就是该3D点的描述子
      cv::Mat mDescriptor; ///< 通过 ComputeDistinctiveDescriptors() 得到的最优描述子
  
      // Reference KeyFrame
      KeyFrame* mpRefKF;
  
      // Tracking counters
      int mnVisible;
      int mnFound;
  
      // Bad flag (we do not currently erase MapPoint from memory)
      bool mbBad;
      MapPoint* mpReplaced;
  
      // Scale invariance distances
      float mfMinDistance;
      float mfMaxDistance;
  
      Map* mpMap;
  
      std::mutex mMutexPos;
      std::mutex mMutexFeatures;
  };
  
