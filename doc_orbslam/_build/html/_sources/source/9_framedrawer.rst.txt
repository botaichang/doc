9.FrameDrawer 
===================

包含更新frame,画frame函数

::

  class FrameDrawer
  {
  public:
      FrameDrawer(Map* pMap);
  
      // Update info from the last processed frame.
      void Update(Tracking *pTracker);
  
      // Draw last processed frame.
      cv::Mat DrawFrame();
  
  protected:
  
      void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
  
      // Info of the frame to be drawn
      cv::Mat mIm;
      int N;
      vector<cv::KeyPoint> mvCurrentKeys;
      vector<bool> mvbMap, mvbVO;
      bool mbOnlyTracking;
      int mnTracked, mnTrackedVO;
      vector<cv::KeyPoint> mvIniKeys;
      vector<int> mvIniMatches;
      int mState;
  
      Map* mpMap;
  
      std::mutex mMutex;
  };
