interface Control {
  limit: number;
  module: boolean;
  autoMode: boolean;
  power: boolean;
  steer: number;
  speed: number;
}

export const controlDefaultParams: Control = {
  limit: 0,
  module: false,
  autoMode: false,
  power: false,
  steer: 0,
  speed: 0,
};

export default Control;
