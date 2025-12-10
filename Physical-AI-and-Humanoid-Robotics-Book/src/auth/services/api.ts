import { UserCreate, UserLogin, AuthUserProfile } from '../types';
export const API_BASE_URL = 'https://hamzasyed001122-auth-backend.hf.space';

export const signup = async (userData: UserCreate) => {
  const response = await fetch(`${API_BASE_URL}/auth/signup`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
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
    headers: {
      'Content-Type': 'application/json',
    },
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

    // Handle unauthenticated or not found (401/404) gracefully
    if (response.status === 401 || response.status === 404) {
      return null;
    }

    // Handle non-JSON errors
    // If response does'nt 200, JSON could'nt be paresed.
    if (!response.ok) {
      const textError = await response.text();
      console.error('Backend text response on error:', textError);
      throw new Error(`Server failed to fetch user (Status: ${response.status})`);
    }

    // When response 200, JSON parse.
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
