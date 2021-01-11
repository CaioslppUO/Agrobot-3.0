import {TouchableOpacity, StyleSheet} from 'react-native';
import React from 'react';
import Icon from 'react-native-vector-icons/FontAwesome';

var col;
export default function button({onPress, name, size = 30, color, style}) {
  return (
    <TouchableOpacity
      style={[Styles.button, style, {borderColor: color}]}
      onPress={onPress}
      delayPressIn={0}>
      <Icon name={name} size={size} color={color} />
    </TouchableOpacity>
  );
}
const Styles = StyleSheet.create({
  button: {
    alignItems: 'center',
    justifyContent: 'center',
    borderWidth: 1.5,
    borderRadius: 180,
    width: 25,
    height: 25,
  },
});
