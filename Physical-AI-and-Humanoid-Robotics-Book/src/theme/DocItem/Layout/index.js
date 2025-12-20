import React from 'react';
import DocItemLayout from '@theme-original/DocItem/Layout';
import useReadingHistoryTracker from '@site/src/hocs/useReadingHistoryTracker';
import { useLocation } from '@docusaurus/router';
import RecommendationPanel from '@site/src/components/RecommendationPanel';
// 🔹 Docusaurus ka BrowserOnly import karein
import BrowserOnly from '@docusaurus/BrowserOnly';

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
          {/* 🔹 RecommendationPanel ko BrowserOnly mein wrap kar dein */}
          <BrowserOnly fallback={<div>Loading recommendations...</div>}>
            {() => <RecommendationPanel />}
          </BrowserOnly>
        </div>
      </div>
    </div>
  );
}