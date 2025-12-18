import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import useAuth from '@site/src/hooks/useAuth';

const API_URL = 'https://hamzasyed001122-content-personalization-backend.hf.space/personalization/preferences';

export default function PersonalizationSettingsPage() {
  const { jwtToken } = useAuth();
  const [preferences, setPreferences] = useState({
    skill_level: 'Beginner',
    interests: [],
    learning_style: 'Practical',
  });
  const [loading, setLoading] = useState(true);
  const [saveLoading, setSaveLoading] = useState(false);
  const [message, setMessage] = useState('');
  const [fetchError, setFetchError] = useState(false); // Naya: Error state tracking

  useEffect(() => {
    const fetchData = async () => {
      const tokenToUse = jwtToken || 'test-token-123';
      setFetchError(false); // Reset error
      
      try {
        const res = await fetch(API_URL, {
          headers: { Authorization: `Bearer ${tokenToUse}` },
        });
        
        // CHECK: Agar backend busy hai ya error de raha hai
        if (!res.ok) {
          throw new Error(`Server error: ${res.status}`);
        }

        const data = await res.json();
        if (data) setPreferences(data);
      } catch (err) {
        console.error('Error fetching preferences:', err);
        setFetchError(true); // Frontend ko pata chalay ga ke error aaya hai
      } finally {
        setLoading(false); 
      }
    };

    fetchData();
  }, [jwtToken]);

  const handleSave = async () => {
    setSaveLoading(true);
    setMessage('');
    try {
      const response = await fetch(API_URL, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${jwtToken || 'test-token-123'}`,
        },
        body: JSON.stringify(preferences),
      });

      if (response.ok) {
        setMessage('✅ Preferences saved successfully!');
      } else {
        // Backend busy hone par ye message aayega
        setMessage('❌ Backend is busy (Database Timeout). Please try again in 1 minute.');
      }
    } catch (err) {
      setMessage('❌ Connection failed. Ensure backend is running.');
    } finally {
      setSaveLoading(false);
    }
  };

  const handleInterestChange = (interest) => {
    setPreferences((prev) => ({
      ...prev,
      interests: prev.interests.includes(interest)
        ? prev.interests.filter((i) => i !== interest)
        : [...prev.interests, interest],
    }));
  };

  if (loading) {
    return (
      <Layout>
        <div style={{ padding: '2rem', textAlign: 'center' }}>
          <h3>Checking your profile...</h3>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Settings">
      <div style={{ padding: '2rem', maxWidth: '600px', margin: '0 auto' }}>
        <h1>Personalization Settings</h1>
        
        {/* Naya: Error Alert agar backend down ho */}
        {fetchError && (
          <div style={{ padding: '10px', backgroundColor: '#fff3cd', color: '#856404', borderRadius: '5px', marginBottom: '20px' }}>
            <strong>Note:</strong> We couldn't load your saved settings from the server. Using default values for now.
          </div>
        )}

        <p>Tailor your learning experience for the Humanoid Robotics book.</p>
        <hr />

        <h3>Skill Level</h3>
        <select 
          value={preferences.skill_level} 
          onChange={(e) => setPreferences({...preferences, skill_level: e.target.value})}
          style={{ padding: '8px', width: '100%' }}
        >
          <option value="Beginner">Beginner</option>
          <option value="Intermediate">Intermediate</option>
          <option value="Expert">Expert</option>
        </select>

        <h3>Interests</h3>
        {['ROS2', 'Computer Vision', 'Control Systems', 'Hardware Design'].map((item) => (
          <div key={item}>
            <input 
              type="checkbox" 
              checked={preferences.interests.includes(item)} 
              onChange={() => handleInterestChange(item)} 
            /> {item}
          </div>
        ))}

        <h3>Learning Style</h3>
        <select 
          value={preferences.learning_style} 
          onChange={(e) => setPreferences({...preferences, learning_style: e.target.value})}
          style={{ padding: '8px', width: '100%' }}
        >
          <option value="Practical">Practical (Hands-on)</option>
          <option value="Theoretical">Theoretical (Math & Logic)</option>
        </select>

        <br /><br />

        <button 
          onClick={handleSave} 
          disabled={saveLoading}
          style={{ 
            padding: '10px 20px', 
            backgroundColor: '#25c2a0', 
            color: 'white', 
            border: 'none', 
            borderRadius: '5px',
            cursor: 'pointer',
            opacity: saveLoading ? 0.7 : 1
          }}
        >
          {saveLoading ? 'Connecting to Database...' : 'Save Preferences'}
        </button>

        {message && <p style={{ marginTop: '1rem', fontWeight: 'bold' }}>{message}</p>}
      </div>
    </Layout>
  );
}