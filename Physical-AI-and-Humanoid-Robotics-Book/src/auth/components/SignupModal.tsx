import React, { useState } from 'react';
import { useAuth } from '../context/AuthProvider';
import { signup } from '../services/api';
import styles from './SignupModal.module.css';

interface SignupModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const SignupModal: React.FC<SignupModalProps> = ({ isOpen, onClose }) => {
  const { openModal } = useAuth();
  const [fullName, setFullName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [skillLevel, setSkillLevel] = useState('');
  const [hardware, setHardware] = useState('');
  const [roboticsExperience, setRoboticsExperience] = useState('');
  const [os, setOs] = useState('');
  const [learningMode, setLearningMode] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);

  if (!isOpen) return null;

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsSubmitting(true);

    try {
      await signup({
        full_name: fullName,
        email,
        password,
        skill_level: skillLevel || undefined,
        hardware: hardware || undefined,
        robotics_experience: roboticsExperience || undefined,
        os: os || undefined,
        learning_mode: learningMode || undefined,
      });

      onClose();
      openModal('login');
    } catch (err: any) {
      setError(err.message || 'An unexpected error occurred during signup.');
    } finally {
      setIsSubmitting(false);
    }
  };

  const switchToLogin = () => {
    onClose();
    openModal('login');
  };

  return (
    <div className={styles.modalBackdrop}>
      <div className={styles.modalContent}>
        <h2>Sign Up</h2>

        {error && <div className={styles.errorText}>{error}</div>}

        <form onSubmit={handleSubmit} className={styles.formLayout}>
          {/* Scrollable Area */}
          <div className={styles.formScrollArea}>
            <div className={styles.formGroup}>
              <label>Full Name:*</label>
              <input
                type="text"
                value={fullName}
                onChange={(e) => setFullName(e.target.value)}
                required
                disabled={isSubmitting}
              />
            </div>

            <div className={styles.formGroup}>
              <label>Email:</label>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                disabled={isSubmitting}
              />
            </div>

            <div className={styles.formGroup}>
              <label>Password:</label>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                disabled={isSubmitting}
              />
            </div>

            <div className={styles.formGroup}>
              <label>Skill Level:</label>
              <select
                value={skillLevel}
                onChange={(e) => setSkillLevel(e.target.value)}
                disabled={isSubmitting}
                required
              >
                <option value="">Select...</option>
                <option value="Beginner">Beginner</option>
                <option value="Intermediate">Intermediate</option>
                <option value="Advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label>Hardware Availability:</label>
              <input
                type="text"
                placeholder="e.g., GPU, Jetson, Raspberry Pi"
                value={hardware}
                onChange={(e) => setHardware(e.target.value)}
                disabled={isSubmitting}
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label>Robotics Experience:</label>
              <input
                type="text"
                value={roboticsExperience}
                onChange={(e) => setRoboticsExperience(e.target.value)}
                disabled={isSubmitting}
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label>Operating System:</label>
              <select value={os} onChange={(e) => setOs(e.target.value)} disabled={isSubmitting}>
                <option value="">Select...</option>
                <option value="Windows">Windows</option>
                <option value="Linux">Linux</option>
                <option value="macOS">macOS</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label>Preferred Learning Mode:</label>
              <select
                value={learningMode}
                onChange={(e) => setLearningMode(e.target.value)}
                disabled={isSubmitting}
                required
              >
                <option value="">Select...</option>
                <option value="Video">Video</option>
                <option value="Text">Text</option>
                <option value="Hands-On">Hands-On</option>
              </select>
            </div>
          </div>

          {/* Buttons */}
          <div className={styles.buttonContainer}>
            <button type="submit" className={styles.signupButton} disabled={isSubmitting}>
              {isSubmitting ? 'Signing Up...' : 'Sign Up'}
            </button>

            <button
              type="button"
              className={styles.cancelButton}
              onClick={onClose}
              disabled={isSubmitting}
            >
              Cancel
            </button>
          </div>
          <div className={styles.switchLink}>
            Already have an account?
            <button type="button" onClick={switchToLogin} className={styles.switchLinkButton}>
              Log In
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};

export default SignupModal;
