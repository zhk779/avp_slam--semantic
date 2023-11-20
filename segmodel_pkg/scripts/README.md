## <div align="center">Usage</div>

<details open>
  <summary><strong>Requirements</strong></summary>

* python >= 3.6
* torch >= 1.8.1
* torchvision >= 0.9.1

Other requirements can be installed with `pip install -r requirements.txt`.

<br>
<details>
  <summary><strong>Training</strong> (click to expand)</summary>

Firstly, you need to configure the path of csv files in the config file. To train with a single GPU:

```bash
$ python tools/train.py --cfg configs/parkingslotsSFNet.yaml
```


</details>

<br>
<details>
  <summary><strong>Evaluation</strong> (click to expand)</summary>

Make sure to set `MODEL_PATH` of the configuration file to your trained model directory.

```bash
$ python tools/val.py --cfg configs/parkingslotsSFNet.yaml.yaml
```

</details>


<br>
<details>
  <summary><strong>Citations</strong> (click to expand)</summary>

```
@inproceedings{sfnet,
  title={Semantic Flow for Fast and Accurate Scene Parsing},
  author={Li, Xiangtai and You, Ansheng and Zhu, Zhen and Zhao, Houlong and Yang, Maoke and Yang, Kuiyuan and Tong, Yunhai},
  booktitle={ECCV},
  year={2020}
}
```

</details>

<br>
<details>
  <summary><strong>References</strong> (click to expand)</summary>

* https://github.com/sithu31296/semantic-segmentation

</details>
