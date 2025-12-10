import React, { JSX } from 'react';
import AuthStatusDisplay from './AuthStatusDisplay';

export default function AuthNavbarItem({
  mobile = false,
  position = 'right',
  ...props
}: any): JSX.Element {
  return (
    <AuthStatusDisplay />
  );
}
