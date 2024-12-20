from dex_retargeting.retargeting_config import RetargetingConfig
from pathlib import Path
import yaml
from enum import Enum
import numpy as np

class HandType(Enum):
    INSPIRE_HAND = "../assets/inspire_hand/inspire_hand.yml"
    UNITREE_DEX3 = "../assets/unitree_hand/unitree_dex3.yml"
    UNITREE_DEX3_Unit_Test = "../../assets/unitree_hand/unitree_dex3.yml"

class HandRetargeting:
    def __init__(self, hand_type: HandType):
        if hand_type == HandType.UNITREE_DEX3:
            RetargetingConfig.set_default_urdf_dir('../assets')
        elif hand_type == HandType.UNITREE_DEX3_Unit_Test:
            RetargetingConfig.set_default_urdf_dir('../../assets')
        elif hand_type == HandType.INSPIRE_HAND:
            RetargetingConfig.set_default_urdf_dir('../assets')

        config_file_path = Path(hand_type.value)

        try:
            with config_file_path.open('r') as f:
                self.cfg = yaml.safe_load(f)
                
            if 'left' not in self.cfg or 'right' not in self.cfg:
                raise ValueError("Configuration file must contain 'left' and 'right' keys.")

            left_retargeting_config = RetargetingConfig.from_dict(self.cfg['left'])
            right_retargeting_config = RetargetingConfig.from_dict(self.cfg['right'])
            self.left_retargeting = left_retargeting_config.build()
            self.right_retargeting = right_retargeting_config.build()
        
        except FileNotFoundError:
            print(f"Configuration file not found: {config_file_path}")
            raise
        except yaml.YAMLError as e:
            print(f"YAML error while reading {config_file_path}: {e}")
            raise
        except Exception as e:
            print(f"An error occurred: {e}")
            raise

if __name__ == "__main__":
    hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
    print(f"hand_retargeting: {dir(hand_retargeting.right_retargeting)}")
    
    hand_mat = [[-0.02358, 0.08363, 0.07629],
                [-0.01324, 0.03749, 0.13984],
                [0.01151, 0.04896, 0.13798],
                [0.03135, 0.05602, 0.12207],
                [0.04432, 0.05533, 0.09699]]
    hand_mat = np.array(hand_mat)
    print(f"hand_mat: {hand_mat}")
    hand_q = hand_retargeting.right_retargeting.retarget(hand_mat)
    print(f"hand_q: {hand_q}")
