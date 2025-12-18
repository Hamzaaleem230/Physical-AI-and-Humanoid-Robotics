import React, { useState, useEffect } from 'react';
import useAuth from '@site/src/hooks/useAuth';
import { useHistory } from '@docusaurus/router';

const PERSONALIZATION_API_BASE_URL =
  'https://hamzasyed001122-content-personalization-backend.hf.space/personalization';

function RecommendationPanel() {
  const { jwtToken } = useAuth();
  const history = useHistory();
  const [recommendations, setRecommendations] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [hasPreferences, setHasPreferences] = useState(true);

  useEffect(() => {
    const fetchRecommendations = async () => {
      // 1. Agar login nahi hai toh kuch fetch na karein
      if (!jwtToken) {
        setLoading(false);
        return;
      }

      setLoading(true);
      setError(null);
      try {
        const response = await fetch(`${PERSONALIZATION_API_BASE_URL}/recommendations`, {
          headers: { Authorization: `Bearer ${jwtToken}` },
        });

        if (response.status === 404) {
          // User exists but no preferences found
          setHasPreferences(false);
          setRecommendations([]);
        } else if (!response.ok) {
          throw new Error(`Server status: ${response.status}`);
        } else {
          const data = await response.json();
          setRecommendations(data.recommendations || []);
          setHasPreferences(true);
        }
      } catch (e) {
        console.error('Error fetching recommendations:', e);
        setError(e.message);
      } finally {
        setLoading(false);
      }
    };

    fetchRecommendations();
  }, [jwtToken]);

  // UI Case 1: User Logged In nahi hai
  if (!jwtToken) {
    return (
      <div style={{ padding: '15px', border: '1px solid #eee', borderRadius: '8px', textAlign: 'center' }}>
        <p>Please <b>Login</b> to see personalized topics.</p>
        <button 
          onClick={() => history.push('/login')}
          style={{ padding: '5px 15px', backgroundColor: '#25c2a0', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
        >
          Login
        </button>
      </div>
    );
  }

  if (loading)
    return <div style={{ padding: '10px', color: '#666' }}>Finding best topics for you...</div>;

  // UI Case 2: User Login hai par Preferences set nahi hain (Popup Style Notice)
  if (!hasPreferences || recommendations.length === 0) {
    return (
      <div style={{ 
        padding: '15px', 
        border: '2px dashed #ffba08', 
        borderRadius: '8px', 
        background: '#fff9e6',
        textAlign: 'center' 
      }}>
        <h4 style={{ margin: '0 0 10px 0', color: '#856404' }}>✨ Personalize Your Journey</h4>
        <p style={{ fontSize: '0.9em' }}>Set your preferences to get AI-powered recommendations.</p>
        <button 
          onClick={() => history.push('/personalization-settings')}
          style={{ padding: '8px 20px', backgroundColor: '#ffba08', color: '#000', border: 'none', borderRadius: '5px', fontWeight: 'bold', cursor: 'pointer' }}
        >
          Go to Settings
        </button>
      </div>
    );
  }

  // UI Case 3: Sab set hai, Recommendations dikhayein
  return (
    <div
      style={{
        padding: '15px',
        border: '1px solid #25c2a0',
        borderRadius: '8px',
        background: 'rgba(37, 194, 160, 0.05)',
      }}
    >
      <h3 style={{ marginTop: 0, color: '#25c2a0' }}>Suggested for You</h3>
      <ul style={{ listStyle: 'none', padding: 0, margin: 0 }}>
        {recommendations.map((item, index) => {
          const id = item.contentId || item.content_id || item.id;
          const title = item.title || 'Humanoid Robotics Topic';
          
          if (!id) return null;

          let finalPath = `/${id}`;
          const pathMap = {
            'what-is-physical-ai': '/modules/ros2-nervous-system/what-is-physical-ai',
            'ros2-architecture': '/modules/ros2-nervous-system/ros2-architecture',
            'building-ros2-packages': '/modules/ros2-nervous-system/building-ros2-packages',
            'launch-files-parameters-tf2': '/modules/ros2-nervous-system/launch-files-parameters-tf2',
            'urdf-for-humanoids': '/modules/ros2-nervous-system/urdf-for-humanoids',
            'motor-commands-sensors-control-loops': '/modules/ros2-nervous-system/motor-commands-sensors-control-loops',
            'ros2-nodes': '/modules/ros2-nervous-system/ros2-architecture',
            'gazebo-setup-physics-engine': '/modules/digital-twin/gazebo-setup-physics-engine',
            'urdf-to-sdf-conversion': '/modules/digital-twin/urdf-to-sdf-conversion',
            'sensor-simulation': '/modules/digital-twin/sensor-simulation',
            'collisions-dynamics-balance-simulation': '/modules/digital-twin/collisions-dynamics-balance-simulation',
            'unity-for-high-fidelity-rendering': '/modules/digital-twin/unity-for-high-fidelity-rendering',
            'ros2-integration-with-simulation': '/modules/digital-twin/ros2-integration-with-simulation',
            'computer-vision-basics': '/modules/digital-twin/sensor-simulation',
            'isaac-sim-omniverse-introduction': '/modules/isaac-brain/isaac-sim-omniverse-introduction',
            'synthetic-data-generation': '/modules/isaac-brain/synthetic-data-generation',
            'isaac-ros-vslam-perception-navigation': '/modules/isaac-brain/isaac-ros-vslam-perception-navigation',
            'nav2-for-humanoid-locomotion': '/modules/isaac-brain/nav2-for-humanoid-locomotion',
            'reinforcement-learning-for-control': '/modules/isaac-brain/reinforcement-learning-for-control',
            'sim-to-real-transfer-design': '/modules/isaac-brain/sim-to-real-transfer-design',
            'whisper-voice-commands-integration': '/modules/vla/whisper-voice-commands-integration',
            'llm-planning-natural-language-to-ros2-tasks': '/modules/vla/llm-planning-natural-language-to-ros2-tasks',
            'multimodal-interaction': '/modules/vla/multimodal-interaction',
            'vla-control-graphs-for-humanoids': '/modules/vla/vla-control-graphs-for-humanoids',
            'full-loop-voice-plan-navigate-perceive-manipulate': '/modules/vla/full-loop-voice-plan-navigate-perceive-manipulate',
          };

          const lowerId = String(id).toLowerCase();
          const lowerTitle = String(item.title || '').toLowerCase();

          if (pathMap[id]) {
            finalPath = pathMap[id];
          } else if (lowerId.includes('ros2') || lowerTitle.includes('ros 2')) {
            finalPath = '/modules/ros2-nervous-system/ros2-architecture';
          } else if (lowerId.includes('vision') || lowerTitle.includes('computer vision')) {
            finalPath = '/modules/digital-twin/sensor-simulation';
          } else if (lowerId.includes('isaac') || lowerTitle.includes('isaac')) {
            finalPath = `/modules/isaac-brain/${id}`;
          }

          return (
            <li key={id || index} style={{ marginBottom: '12px' }}>
              <a href={finalPath} style={{ fontWeight: 'bold', color: '#25c2a0', textDecoration: 'none' }}>
                {title}
              </a>
              <div style={{ fontSize: '0.75em', color: '#888' }}>
                Relevance: {item.score ? Math.round(item.score * 100) : '95'}%
              </div>
            </li>
          );
        })}
      </ul>
    </div>
  );
}

export default RecommendationPanel;