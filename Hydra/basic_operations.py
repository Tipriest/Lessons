import hydra
from omegaconf import dictconfig


# @hydra.conf()
# def dynamic_config():
#     cfg = {
#         training:{
#             batch_size:10
#         }
#     }
#     return cfg


@hydra.main(config_path="configs", config_name="train_config.yaml", version_base="1.1")
def my_app(cfg: dictconfig):
    print(cfg)


if __name__ == "__main__":
    my_app()
