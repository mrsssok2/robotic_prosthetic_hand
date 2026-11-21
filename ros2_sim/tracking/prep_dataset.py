"""Write data/hand-keypoints.yaml with file lists: train (official), val (4992, model selection), test (3000, held out).

The official val split (7992) is cut deterministically so the checkpoint is not picked on the images used for
the reported numbers. flip_idx is the identity: mirroring a hand image gives a hand of the other chirality whose
landmarks keep the same meaning (the yaml shipped with the dataset swaps indices, which mislabels flipped hands).
"""
import pathlib
import random

root = pathlib.Path(__file__).resolve().parents[1] / "data"
ds = root / "hand-keypoints"
val = sorted((ds / "images" / "val").glob("*.jpg"))
random.Random(0).shuffle(val)
test, val_sel = val[:3000], val[3000:]
(root / "train.txt").write_text("\n".join(map(str, sorted((ds / "images" / "train").glob("*.jpg")))) + "\n")
(root / "val_sel.txt").write_text("\n".join(map(str, sorted(val_sel))) + "\n")
(root / "test.txt").write_text("\n".join(map(str, sorted(test))) + "\n")
(root / "hand-keypoints.yaml").write_text(f"""path: {root}
train: {root}/train.txt
val: {root}/val_sel.txt
test: {root}/test.txt
kpt_shape: [21, 3]
flip_idx: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
names:
  0: hand
""")
print(len(val_sel), len(test))
