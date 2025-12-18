import React from 'react';
import DocItemLayout from '@theme-original/DocItem/Layout';
import useReadingHistoryTracker from '@site/src/hocs/useReadingHistoryTracker';
import { useLocation } from '@docusaurus/router';
import RecommendationPanel from '@site/src/components/RecommendationPanel';

export default function DocItemLayoutWrapper(props) {
  const location = useLocation();
  
  // URL se contentId nikalna
  const contentId = location.pathname.split('/').pop() || 'home';

  // Har document page par tracking shuru karein
  useReadingHistoryTracker(contentId);

  return (
    <div className="row">
      <div className="col col--9">
        <DocItemLayout {...props} />
      </div>
      {/* Side mein recommendations dikhane ke liye */}
      <div className="col col--3">
        <div style={{ position: 'sticky', top: '100px', padding: '10px' }}>
          <RecommendationPanel />
        </div>
      </div>
    </div>
  );
}