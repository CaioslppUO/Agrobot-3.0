module.exports = {
  root: true,
  extends: ['@react-native-community', 'prettier'],
  rules: {
    'react/jsx-filename-extension': ['error', {extensions: ['.js', '.jsx']}],
    'global-require': 'off',
    'import/prefer-default-export': 'off',
    'no-unused-expressions': ['error', {allowTaggedTemplates: true}],
  },
};
