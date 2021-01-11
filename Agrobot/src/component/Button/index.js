import {Text, TouchableOpacity, StyleSheet} from 'react-native';
import React from 'react';

export default function button({onPress, style, styleText, children}) {
  return (
    <TouchableOpacity
      delayPressIn={0}
      style={[Styles.button, style]}
      onPress={onPress}>
      <Text style={[Styles.text, styleText]}>{children}</Text>
    </TouchableOpacity>
  );
}

const Styles = StyleSheet.create({
  text: {
    fontSize: 25,
    textAlign: 'center',
    borderRadius: 10,
  },
  button: {
    alignContent: 'center',
    justifyContent: 'center',
    width: 140,
    height: 90,
    borderColor: '#314159',
    borderWidth: 3,
    borderRadius: 20,
  },
});
