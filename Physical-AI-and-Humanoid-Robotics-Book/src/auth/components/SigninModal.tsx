import React, { useState } from 'react';
import { useAuth } from '../context/AuthProvider';
import styles from './SigninModal.module.css'; // 💡 CSS Module Import

interface SigninModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const SigninModal: React.FC<SigninModalProps> = ({ isOpen, onClose }) => {
  const { login, openModal } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);

  if (!isOpen) return null;

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsSubmitting(true);
    try {
      await login(email, password);
      onClose();
    } catch (err: any) {
      setError(err.message || 'An unexpected error occurred during signin.');
    } finally {
      setIsSubmitting(false);
    }
  };

  const switchToSignup = () => {
    onClose();
    openModal('signup');
  };

  return (
    <div className={styles.modalBackdrop}> 
      <div className={styles.modalContent}> 
        <h2>Sign In</h2>
        {error && <div className={styles.errorText}>{error}</div>} 
        
        <form onSubmit={handleSubmit}>
          <div className={styles.formGroup}> 
            <label htmlFor="email">Email:</label>
            <input
              id="email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              disabled={isSubmitting}
            />
          </div>
          
          <div className={styles.formGroup}> 
            <label htmlFor="password">Password:</label>
            <input
              id="password"
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              disabled={isSubmitting}
            />
          </div>
          
          <div className={styles.buttonContainer}> 
            <button 
              type="submit" 
              className={styles.signinButton} 
              disabled={isSubmitting}
            >
              {isSubmitting ? 'Signing In...' : 'Sign In'}
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
        </form>
        
        <div className={styles.switchLink}> 
          Don't have an account? 
          <button 
            type="button" 
            onClick={switchToSignup}
            className={styles.switchLinkButton} 
          >
            Sign Up
          </button>
        </div>
      </div>
    </div>
  );
};

export default SigninModal;