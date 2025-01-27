# -------------------------- Train on Single-GPU --------------------------
python train.py \
        --cuda \
        -d coco \
        --root /data/datasets/ \
        -m yolox_s \
        -bs 8 \
        -size 640 \
        --wp_epoch 3 \
        --max_epoch 300 \
        --eval_epoch 10 \
        --no_aug_epoch 15 \
        --ema \
        --fp16 \
        --multi_scale \
        # --load_cache \
        # --resume weights/coco/yolox_m/yolox_m_best.pth \
        # --eval_first
