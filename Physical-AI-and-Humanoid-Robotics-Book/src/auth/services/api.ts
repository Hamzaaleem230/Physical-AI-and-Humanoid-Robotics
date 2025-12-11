import { UserCreate, UserLogin, AuthUserProfile } from '../types';

export const API_BASE_URL = 'https://hamzasyed001122-auth-backend.hf.space';

export const signup = async (userData: UserCreate) => {
  const response = await fetch(`${API_BASE_URL}/auth/signup`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(userData),
  });

  if (!response.ok) {
    const errorData = await response.json();
    throw new Error(errorData.detail || 'Failed to sign up');
  }

  return response.json();
};

export const signin = async (credentials: UserLogin) => {
  const response = await fetch(`${API_BASE_URL}/auth/signin`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(credentials),
    credentials: 'include',
  });

  if (!response.ok) {
    const errorData = await response.json();
    throw new Error(errorData.detail || 'Failed to sign in');
  }

  return response.json();
};

export const getCurrentUser = async (): Promise<AuthUserProfile | null> => {
  try {
    const response = await fetch(`${API_BASE_URL}/auth/me`, {
      credentials: 'include',
    });

    if (response.status === 401 || response.status === 404) return null;

    if (!response.ok) {
      const raw = await response.text();
      console.error('Backend error:', raw);
      return null;
    }

    return response.json();
  } catch (error) {
    console.error('Error fetching current user:', error);
    return null;
  }
};

export const signout = async () => {
  const response = await fetch(`${API_BASE_URL}/auth/signout`, {
    method: 'POST',
    credentials: 'include',
  });

  if (!response.ok) {
    const errorData = await response.json();
    throw new Error(errorData.detail || 'Failed to sign out');
  }

  return response.json();
};
