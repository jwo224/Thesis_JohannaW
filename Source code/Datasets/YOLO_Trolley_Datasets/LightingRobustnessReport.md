# Lighting Robustness Evaluation of YOLOv8 Trolley Detection

## Experimental setup

Four YOLOv8s trolley-detection models were evaluated:

1. Left-camera dataset without augmentation
2. Left-camera dataset with brightness augmentation
3. Full dataset without augmentation
4. Full dataset with brightness augmentation

The models detect three classes: trash trolley, laundry trolley, and empty trolley. The augmented models were trained with brightness variation using `hsv_v=0.40`. Hue, saturation, and geometric augmentations were disabled so that the effect of brightness augmentation could be examined independently.

A common holdout set was created from the left-only validation split. The original split contained 59 images, but 22 had also been used to train the full-dataset models. These images were excluded, leaving **37 images that were not used to train any of the four models**.

Each holdout image was evaluated under five conditions:

- Clean, unchanged image
- Bright windows, simulated with 1.6× brightness
- Dark corridors, simulated with 0.45× brightness
- Artificial partial shadows
- Darker, warm evening illumination

The principal metric was mAP50–95.

## Results

| Model | Clean | Bright | Dark | Shadows | Evening | Mean altered |
|---|---:|---:|---:|---:|---:|---:|
| Left / No augmentation | 0.953 | **0.957** | 0.924 | 0.948 | **0.930** | **0.940** |
| Left / Brightness | 0.931 | 0.923 | 0.905 | 0.922 | 0.923 | 0.918 |
| Full / No augmentation | **0.986** | 0.849 | 0.852 | 0.920 | 0.416 | 0.759 |
| Full / Brightness | 0.961 | 0.903 | **0.942** | **0.968** | 0.572 | 0.846 |

## Discussion

The left-only model trained without augmentation provided the strongest overall lighting robustness. It achieved a clean mAP50–95 of 0.953 and a mean of 0.940 across the four altered conditions. Its worst result was still 0.924 under the dark-corridor transformation, and it retained approximately 98.6% of its clean performance on average.

Brightness augmentation did not improve the left-only model. Its clean score decreased from 0.953 to 0.931, while its mean altered-lighting score decreased from 0.940 to 0.918. Although the augmented model remained stable, its absolute score was lower in every tested condition. For this dataset, `hsv_v=0.40` may have been stronger than necessary, or the original left-camera training data may already have contained sufficient lighting variation.

The full model without augmentation achieved the highest clean score, at 0.986, but was considerably less robust to lighting changes. Its performance fell to 0.849 in bright conditions, 0.852 in dark conditions, and 0.416 under evening illumination.

Brightness augmentation clearly improved the robustness of the full-dataset model. Compared with the full baseline, its mAP50–95 increased:

- From 0.849 to 0.903 under bright lighting
- From 0.852 to 0.942 under dark lighting
- From 0.920 to 0.968 under shadows
- From 0.416 to 0.572 under evening lighting

Its mean altered-lighting score therefore increased from 0.759 to 0.846. However, the full brightness-augmented model still performed substantially worse than both left-only models in the evening condition.

The empty-trolley class was the main evening-lighting failure. Its AP50–95 fell to zero for both full-dataset models. This suggests that brightness augmentation alone does not adequately represent the combined changes in brightness, contrast, and colour temperature introduced by the evening transformation.

## Conclusion

For a system using the left camera, the left-only YOLOv8s model trained without augmentation was the best model in this experiment. It produced the highest average altered-lighting performance and remained consistently strong across every condition.

Brightness augmentation was beneficial for the full-dataset model, especially under dark and shadowed conditions, but it did not outperform the left-only baseline overall. The results indicate that dataset and camera-domain consistency had a greater effect than the selected brightness augmentation.

Future work should test a smaller brightness range, such as `hsv_v=0.20–0.30`, and restrained colour or temperature augmentation for evening conditions. Evaluation should also be repeated using independently captured real-world lighting images.

## Limitations

The shared holdout contains only 37 images, so the results should be interpreted cautiously. In addition, the lighting conditions were generated synthetically from the same images. The experiment measures sensitivity to controlled lighting transformations but does not fully represent generalization to new physical environments, camera positions, or recording sessions.
