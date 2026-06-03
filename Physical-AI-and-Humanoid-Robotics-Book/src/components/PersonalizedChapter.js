import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

function PersonalizedChapterContent() {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [userLevel, setUserLevel] = useState('Beginner');

  useEffect(() => {
    // 🔹 Important: LocalStorage mein aapka data 'authUser' key mein hai
    const authData = localStorage.getItem('authUser');
    if (authData) {
      try {
        const parsedData = JSON.parse(authData);
        // 🔹 Aapke screenshot ke mutabiq key ka naam 'skill_level' hai
        if (parsedData.skill_level) {
          setUserLevel(parsedData.skill_level);
        }
      } catch (e) {
        console.error("Error parsing authUser data", e);
      }
    }
  }, []);

  const togglePersonalization = () => setIsPersonalized(!isPersonalized);

  return (
    <div style={{
      margin: '20px 0',
      padding: '20px',
      borderRadius: '12px',
      background: isPersonalized ? '#e3f2fd' : '#f5f5f5',
      border: '2px solid',
      borderColor: isPersonalized ? '#2196f3' : '#ddd',
      transition: 'all 0.3s ease',
      color: isPersonalized ? '#1565c0' : '#333', // Blue color when active for better contrast
    }}>
      {/* Top Header Section */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <h4 style={{ margin: 0 }}>✨ Chapter Personalization</h4>
        <button 
          onClick={togglePersonalization}
          className={`button button--${isPersonalized ? 'primary' : 'secondary'}`}
        >
          {isPersonalized ? 'Showing Personalized View' : 'Personalize this Chapter'}
        </button>
      </div>

      <div style={{ marginTop: '15px' }}>
        {/* State 1: Before Clicking Button */}
        {!isPersonalized ? (
          <p><em>Note: This is the standard view. Click the button to adapt the content to your <strong>{userLevel}</strong> level.</em></p>
        ) : (
          /* State 2: After Clicking Button (The Real Content Switch) */
          <div className="fade-in" style={{ borderTop: '1px solid #bbdefb', paddingTop: '15px' }}>
            <span className="badge badge--success" style={{ marginBottom: '10px' }}>
              Level: {userLevel}
            </span>

            {/* Beginner Content */}
            {userLevel === 'Beginner' && (
              <div>
                <p>🤖 <strong>Basic Overview:</strong> Welcome! In this chapter, we will learn how robots move and think. We'll focus on how sensors work like human eyes and motors work like muscles, keeping it simple and math-free.</p>
                <ul>
                  <li>Introduction to Robot parts</li>
                  <li>How to make a robot move forward</li>
                </ul>
              </div>
            )}

            {/* Intermediate Content */}
            {userLevel === 'Intermediate' && (
              <div>
                <p>⚙️ <strong>Technical Depth:</strong> We are diving into ROS 2 nodes and communication protocols. You will learn about <code>rclpy</code> and how to manage publishers and subscribers for humanoid balance.</p>
                <ul>
                  <li>ROS 2 Topic Management</li>
                  <li>Sensor Fusion basics</li>
                </ul>
              </div>
            )}

            {/* Advanced / Expert Content */}
            {(userLevel === 'Advanced' || userLevel === 'Expert') && (
              <div>
                <p>🚀 <strong>Expert Specification:</strong> This module now enables Non-linear Model Predictive Control (NMPC) analysis. We will focus on Zero Moment Point (ZMP) trajectories for bipedal locomotion.</p>
                <ul>
                  <li>Advanced Kinematics & Dynamics</li>
                  <li>Sim-to-Real Transfer optimization</li>
                </ul>
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
}

export default function PersonalizedChapter() {
  return (
    <BrowserOnly fallback={<div>Loading personalization...</div>}>
      {() => <PersonalizedChapterContent />}
    </BrowserOnly>
  );
}