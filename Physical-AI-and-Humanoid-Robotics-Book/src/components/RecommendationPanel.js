import React, { useState, useEffect } from 'react';
import { useAuth } from '../auth/context/AuthProvider';
import { useHistory } from '@docusaurus/router';

const PERSONALIZATION_API_BASE_URL =
  'https://hamzasyed001122-content-personalization-backend.hf.space/personalization';

function RecommendationPanel() {
  const { jwtToken, openModal, user } = useAuth(); 
  const history = useHistory();

  const [recommendations, setRecommendations] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [hasPreferences, setHasPreferences] = useState(true);

  useEffect(() => {
    const fetchRecommendations = async () => {
      const activeToken = jwtToken || localStorage.getItem('jwtToken');

      if (!activeToken) {
        setLoading(false);
        return;
      }

      setLoading(true);
      setError(null);
      try {
        // FIX: cache: 'no-store' add kiya hai taake settings ke baad purana data na dikhaye
        const response = await fetch(`${PERSONALIZATION_API_BASE_URL}/recommendations`, {
          method: 'GET',
          headers: { 
            Authorization: `Bearer ${activeToken}`,
            'Cache-Control': 'no-cache'
          },
          cache: 'no-store' 
        });

        if (response.status === 404) {
          setHasPreferences(false);
          setRecommendations([]);
        } else if (!response.ok) {
          throw new Error(`Server status: ${response.status}`);
        } else {
          const data = await response.json();
          console.log("RECO_DEBUG:", data); // Ye line add karein
          // Agar data mil gaya toh update karein
          if (data.recommendations && data.recommendations.length > 0) {
            setRecommendations(data.recommendations);
            setHasPreferences(true);
          } else {
            // Agar backend se abhi processing chal rahi ho toh
            setHasPreferences(false);
          }
        }
      } catch (e) {
        console.error('Error fetching recommendations:', e);
        setError(e.message);
      } finally {
        setLoading(false);
      }
    };

    fetchRecommendations();
    // dependency array mein user aur jwtToken dono hain taake state change par trigger ho
  }, [jwtToken, user]); 

  // UI Case 1: Not Logged In
  if (!jwtToken && !user && !localStorage.getItem('jwtToken')) {
    return (
      <div style={{ padding: '20px', border: '1px solid #ffba08', borderRadius: '12px', textAlign: 'center', background: 'rgba(255, 186, 8, 0.1)', color: '#fff' }}>
        <p style={{ marginBottom: '15px' }}>Please <b>Sign In</b> to see personalized topics.</p>
        <button 
          onClick={() => openModal('login')} 
          style={{ padding: '8px 20px', backgroundColor: '#25c2a0', color: '#180909ff', border: 'none', borderRadius: '8px', cursor: 'pointer', fontWeight: 'bold' }}
        >
          Sign In
        </button>
      </div>
    );
  }

  if (loading)
    return <div style={{ padding: '20px', color: '#25c2a0', textAlign: 'center' }}>✨ Finding best topics for you...</div>;

  // UI Case 2: No Preferences (Go to Settings)
  if (!hasPreferences || recommendations.length === 0) {
    return (
      <div style={{ padding: '20px', border: '1px solid #ffba08', borderRadius: '12px', background: 'rgba(255, 186, 8, 0.1)', textAlign: 'center' }}>
        <h4 style={{ margin: '0 0 10px 0', color: '#ffba08' }}>✨ Personalize Your Journey</h4>
        <p style={{ fontSize: '0.9em', color: '#eee', marginBottom: '15px' }}>Set your preferences to get AI-powered recommendations.</p>
        <button 
          onClick={() => history.push('/personalization-settings')}
          style={{ padding: '8px 20px', backgroundColor: '#25c2a0', color: '#180909ff', border: 'none', borderRadius: '8px', cursor: 'pointer', transition: 'all 0.2s ease-in', fontWeight: 'bold' }}
        >
          Go to Settings
        </button>
      </div>
    );
  }

  // UI Case 3: Success Display
  return (
    <div style={{ padding: '20px', border: '1px solid #25c2a0', borderRadius: '12px', background: '#1b1b1d' }}>
      <h3 style={{ marginTop: 0, color: '#25c2a0', borderBottom: '1px solid #333', paddingBottom: '10px' }}>Suggested for You</h3>
      <ul style={{ listStyle: 'none', padding: 0, margin: '15px 0 0 0' }}>
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
            <li key={id || index} style={{ marginBottom: '15px', padding: '12px', borderRadius: '8px', background: 'rgba(37, 194, 160, 0.05)', border: '1px solid #333' }}>
              <a href={finalPath} style={{ fontWeight: 'bold', color: '#25c2a0', textDecoration: 'none', display: 'block' }}>
                {title}
              </a>
              <div style={{ fontSize: '0.8em', color: '#888', marginTop: '5px' }}>
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