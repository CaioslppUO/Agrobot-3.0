import {StyleSheet, Dimensions} from 'react-native';

const widthScreen = Dimensions.get('window').width;
const heightScreen = Dimensions.get('window').height;

export default StyleSheet.create({
  title: {
    fontSize: 30,
    textShadowColor: '#999',
    textShadowOffset: {width: 1, height: 2},
    textShadowRadius: 2,
  },
  container: {
    flex: 1,
    alignItems: 'center',
    flexDirection: 'column',
    justifyContent: 'space-around',
    backgroundColor: '#fff',
  },
  inputText: {
    borderWidth: 0,
    width: widthScreen * 0.8,
    marginBottom: heightScreen * 0.01,
    borderRadius: 15,
    shadowOffset: {
      width: 3,
      height: 3,
    },
    shadowOpacity: 0.3,
    shadowRadius: 5,
    elevation: 2,
  },
  buttonSave: {
    width: widthScreen * 0.5,
    height: heightScreen * 0.1,
    shadowRadius: 5,
    elevation: 2,
  },
  textButtonSave: {
    fontSize: 15,
  },
});
