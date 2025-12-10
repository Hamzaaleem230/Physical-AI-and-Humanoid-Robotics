import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import {
  getCurrentUser,
  signout as apiSignout,
  signin as apiSignin,
  signup as apiSignup,
} from '../services/api';
import { AuthUserProfile, UserCreate, UserLogin } from '../types';

// Import Modals for rendering
import SigninModal from '../components/SigninModal';
import SignupModal from '../components/SignupModal';

// Shape of our authentication context
export interface AuthContextType {
  user: AuthUserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  // Modal State and Control functions
  openModal: (type: 'login' | 'signup') => void;
  closeModal: () => void;
}

export const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<AuthUserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const isAuthenticated = !!user;

  // Modal State
  const [isSigninOpen, setIsSigninOpen] = useState(false);
  const [isSignupOpen, setIsSignupOpen] = useState(false);

  useEffect(() => {
    const loadUser = async () => {
      try {
        const currentUser = await getCurrentUser();
        setUser(currentUser);
      } catch (error) {
        console.error('Failed to load user:', error);
        setUser(null);
      } finally {
        setIsLoading(false);
      }
    };
    loadUser();
  }, []);

  const login = async (email: string, password: string) => {
    setIsLoading(true);
    try {
      await apiSignin({ email, password });
      const currentUser = await getCurrentUser();
      setUser(currentUser);
      closeModal(); // Model close when login success
    } catch (error) {
      console.error('Login failed:', error);
      setUser(null);
      throw error;
    } finally {
      setIsLoading(false);
    }
  }; // openModal function

  const openModal = (type: 'login' | 'signup') => {
    closeModal(); // Existing modal band karein
    if (type === 'login') {
      setIsSigninOpen(true);
    } else if (type === 'signup') {
      setIsSignupOpen(true);
    }
  }; // closeModal function

  const closeModal = () => {
    setIsSigninOpen(false);
    setIsSignupOpen(false);
  };

  const logout = async () => {
    setIsLoading(true);
    try {
      await apiSignout();
      setUser(null);
    } catch (error) {
      console.error('Logout failed:', error);
      throw error;
    } finally {
      setIsLoading(false);
    }
  };

  const contextValue: AuthContextType = {
    user,
    isAuthenticated,
    isLoading,
    login,
    logout,
    // Add to context
    openModal,
    closeModal,
  };

  return (
    <AuthContext.Provider value={contextValue}>
            {children}
      {/* New Models Render */}
            <SigninModal isOpen={isSigninOpen} onClose={closeModal} />
            <SignupModal isOpen={isSignupOpen} onClose={closeModal} />   {' '}
    </AuthContext.Provider>
  );
};

// Custom hook for consuming the context
export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
