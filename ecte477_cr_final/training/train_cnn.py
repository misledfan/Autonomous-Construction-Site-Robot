#!/usr/bin/env python3
"""
train_cnn.py - trains the custom object CNN (bonus 17b) and exports it to ONNX.

Pure NumPy (no PyTorch/TensorFlow needed) so it runs on any lab PC:
    python3 train_cnn.py --data data --out ../models/object_cnn.onnx

Dataset layout (grayscale or colour PNG/JPG crops, any size):
    data/background/*.png     <- anything that is NOT an object (wall, paper, yellow, shapes...)
    data/light_bulb/*.png     <- crops of light bulbs
  Any extra folder = extra class (e.g. data/wrench/). 'background' must exist.

Network (input 1x40x40 grayscale, values 0..1):
    Conv3x3(8)+ReLU+MaxPool2 -> Conv3x3(16)+ReLU+MaxPool2 -> Conv3x3(16)+ReLU+MaxPool2
    -> Flatten(400) -> Dense(32)+ReLU -> Dense(n_classes) -> Softmax
Training: heavy augmentation (rotation, scale, blur, low-resolution, brightness,
noise, JPEG) because the robot camera (320x240 C270) is much worse than a phone.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

SIZE = 40
rng = np.random.default_rng(0)


# ------------------------------------------------------------------ data
def load_folder(path):
    imgs = []
    for f in sorted(glob.glob(os.path.join(path, '*'))):
        im = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if im is not None and im.size:
            imgs.append(im)
    return imgs


def augment(img):
    h, w = img.shape
    # random affine: rotation, scale, shift
    ang = rng.uniform(-30, 30)
    sc = rng.uniform(0.8, 1.2)
    M = cv2.getRotationMatrix2D((w / 2, h / 2), ang, sc)
    M[:, 2] += rng.uniform(-0.08, 0.08, 2) * np.array([w, h])
    border = int(np.median(np.concatenate([img[0], img[-1], img[:, 0], img[:, -1]])))
    im = cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_CONSTANT, borderValue=border)
    if rng.random() < 0.5:
        im = cv2.flip(im, 1)
    # low resolution like the robot camera (object only 15-40 px tall)
    lo = int(rng.integers(14, 41))
    im = cv2.resize(cv2.resize(im, (lo, lo), interpolation=cv2.INTER_AREA), (SIZE, SIZE))
    im = im.astype(np.float32)
    # brightness / contrast
    im = (im - 128) * rng.uniform(0.6, 1.4) + 128 + rng.uniform(-45, 45)
    if rng.random() < 0.5:
        k = rng.choice([3, 5])
        im = cv2.GaussianBlur(im, (k, k), 0)
    im += rng.normal(0, rng.uniform(0, 10), im.shape)
    im = np.clip(im, 0, 255).astype(np.uint8)
    if rng.random() < 0.5:
        ok, enc = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, int(rng.integers(30, 90))])
        im = cv2.imdecode(enc, cv2.IMREAD_GRAYSCALE)
    return im.astype(np.float32) / 255.0


def make_set(classes, per_class):
    X, y = [], []
    for ci, imgs in enumerate(classes):
        for _ in range(per_class):
            X.append(augment(imgs[rng.integers(len(imgs))]))
            y.append(ci)
    X = np.stack(X)[:, None]
    y = np.array(y)
    p = rng.permutation(len(y))
    return X[p], y[p]


# ------------------------------------------------------------- layers
def im2col(x, k=3, pad=1):
    n, c, h, w = x.shape
    xp = np.pad(x, ((0, 0), (0, 0), (pad, pad), (pad, pad)))
    s = xp.strides
    cols = np.lib.stride_tricks.as_strided(
        xp, (n, c, k, k, h, w), (s[0], s[1], s[2], s[3], s[2], s[3]))
    return cols.transpose(0, 4, 5, 1, 2, 3).reshape(n * h * w, c * k * k)


def col2im(cols, shape, k=3, pad=1):
    n, c, h, w = shape
    cols = cols.reshape(n, h, w, c, k, k).transpose(0, 3, 4, 5, 1, 2)
    xp = np.zeros((n, c, h + 2 * pad, w + 2 * pad), cols.dtype)
    for i in range(k):
        for j in range(k):
            xp[:, :, i:i + h, j:j + w] += cols[:, :, i, j]
    return xp[:, :, pad:pad + h, pad:pad + w]


class Conv:
    def __init__(self, cin, cout):
        self.W = rng.normal(0, np.sqrt(2.0 / (cin * 9)), (cout, cin, 3, 3)).astype(np.float32)
        self.b = np.zeros(cout, np.float32)

    def fwd(self, x):
        self.shape = x.shape
        self.cols = im2col(x)
        n, _, h, w = x.shape
        out = self.cols @ self.W.reshape(len(self.b), -1).T + self.b
        return out.reshape(n, h, w, -1).transpose(0, 3, 1, 2)

    def bwd(self, g):
        n, co, h, w = g.shape
        g2 = g.transpose(0, 2, 3, 1).reshape(-1, co)
        self.dW = (g2.T @ self.cols).reshape(self.W.shape)
        self.db = g2.sum(0)
        dcols = g2 @ self.W.reshape(co, -1)
        return col2im(dcols, self.shape)


class Relu:
    def fwd(self, x):
        self.m = x > 0
        return x * self.m

    def bwd(self, g):
        return g * self.m


class Pool:
    def fwd(self, x):
        n, c, h, w = x.shape
        r = x.reshape(n, c, h // 2, 2, w // 2, 2)
        out = r.max(axis=(3, 5))
        self.mask = (r == out[:, :, :, None, :, None])
        self.shape = x.shape
        return out

    def bwd(self, g):
        n, c, h, w = self.shape
        gg = self.mask * g[:, :, :, None, :, None]
        return gg.reshape(n, c, h, w)


class Dense:
    def __init__(self, i, o):
        self.W = rng.normal(0, np.sqrt(2.0 / i), (o, i)).astype(np.float32)
        self.b = np.zeros(o, np.float32)

    def fwd(self, x):
        self.x = x
        return x @ self.W.T + self.b

    def bwd(self, g):
        self.dW = g.T @ self.x
        self.db = g.sum(0)
        return g @ self.W


class Flatten:
    def fwd(self, x):
        self.shape = x.shape
        return x.reshape(len(x), -1)

    def bwd(self, g):
        return g.reshape(self.shape)


def build(n_classes):
    return [Conv(1, 8), Relu(), Pool(), Conv(8, 16), Relu(), Pool(), Conv(16, 16), Relu(), Pool(),
            Flatten(), Dense(16 * 5 * 5, 32), Relu(), Dense(32, n_classes)]


def forward(net, x):
    for l in net:
        x = l.fwd(x)
    return x


def softmax(z):
    z = z - z.max(1, keepdims=True)
    e = np.exp(z)
    return e / e.sum(1, keepdims=True)


def train(net, X, y, Xv, yv, epochs=12, bs=64, lr=2e-3):
    params = [l for l in net if hasattr(l, 'W')]
    m = {id(l): [np.zeros_like(l.W), np.zeros_like(l.b)] for l in params}
    v = {id(l): [np.zeros_like(l.W), np.zeros_like(l.b)] for l in params}
    t = 0
    for ep in range(epochs):
        p = rng.permutation(len(y))
        loss_sum = 0.0
        for i in range(0, len(y), bs):
            idx = p[i:i + bs]
            prob = softmax(forward(net, X[idx]))
            loss_sum += -np.log(prob[np.arange(len(idx)), y[idx]] + 1e-9).sum()
            g = prob
            g[np.arange(len(idx)), y[idx]] -= 1
            g /= len(idx)
            for l in reversed(net):
                g = l.bwd(g)
            t += 1
            for l in params:
                for j, (par, grad) in enumerate(((l.W, l.dW), (l.b, l.db))):
                    mm, vv = m[id(l)][j], v[id(l)][j]
                    mm *= 0.9
                    mm += 0.1 * grad
                    vv *= 0.999
                    vv += 0.001 * grad * grad
                    par -= lr * (mm / (1 - 0.9 ** t)) / (np.sqrt(vv / (1 - 0.999 ** t)) + 1e-8)
        acc = (predict(net, Xv) == yv).mean()
        print(f'epoch {ep + 1:2d}  loss {loss_sum / len(y):.4f}  val_acc {acc:.3f}', flush=True)
    return acc


def predict(net, X, bs=256):
    return np.concatenate([forward(net, X[i:i + bs]).argmax(1) for i in range(0, len(X), bs)])


# ---------------------------------------------------------------- export
def export_onnx(net, path, n_classes):
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    convs = [l for l in net if isinstance(l, Conv)]
    dens = [l for l in net if isinstance(l, Dense)]
    inits, nodes = [], []
    x = 'input'
    for i, c in enumerate(convs):
        inits += [numpy_helper.from_array(c.W.astype(np.float32), f'c{i}_W'),
                  numpy_helper.from_array(c.b.astype(np.float32), f'c{i}_b')]
        nodes += [helper.make_node('Conv', [x, f'c{i}_W', f'c{i}_b'], [f'c{i}'],
                                   kernel_shape=[3, 3], pads=[1, 1, 1, 1]),
                  helper.make_node('Relu', [f'c{i}'], [f'r{i}']),
                  helper.make_node('MaxPool', [f'r{i}'], [f'p{i}'], kernel_shape=[2, 2],
                                   strides=[2, 2])]
        x = f'p{i}'
    nodes.append(helper.make_node('Flatten', [x], ['flat'], axis=1))
    x = 'flat'
    for i, d in enumerate(dens):
        inits += [numpy_helper.from_array(d.W.astype(np.float32), f'd{i}_W'),
                  numpy_helper.from_array(d.b.astype(np.float32), f'd{i}_b')]
        nodes.append(helper.make_node('Gemm', [x, f'd{i}_W', f'd{i}_b'], [f'd{i}'], transB=1))
        x = f'd{i}'
        if i < len(dens) - 1:
            nodes.append(helper.make_node('Relu', [x], [f'dr{i}']))
            x = f'dr{i}'
    nodes.append(helper.make_node('Softmax', [x], ['prob'], axis=1))
    graph = helper.make_graph(
        nodes, 'object_cnn',
        [helper.make_tensor_value_info('input', TensorProto.FLOAT, [1, 1, SIZE, SIZE])],
        [helper.make_tensor_value_info('prob', TensorProto.FLOAT, [1, n_classes])], inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 11)])
    model.ir_version = 6
    onnx.checker.check_model(model)
    onnx.save(model, path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default='data')
    ap.add_argument('--out', default='../models/object_cnn.onnx')
    ap.add_argument('--per-class', type=int, default=1500)
    ap.add_argument('--epochs', type=int, default=12)
    a = ap.parse_args()

    names = sorted(d for d in os.listdir(a.data) if os.path.isdir(os.path.join(a.data, d)))
    if 'background' not in names:
        sys.exit('data/background is required')
    names.remove('background')
    names = ['background'] + names            # index 0 = background
    classes = [load_folder(os.path.join(a.data, n)) for n in names]
    for n, c in zip(names, classes):
        print(f'class {n}: {len(c)} source images')
        if not c:
            sys.exit(f'no images in {n}')
    X, y = make_set(classes, a.per_class)
    Xv, yv = make_set(classes, a.per_class // 5)          # fresh augmentations = validation
    net = build(len(names))
    acc = train(net, X, y, Xv, yv, epochs=a.epochs)

    # confusion matrix on validation set
    pred = predict(net, Xv)
    cm = np.zeros((len(names), len(names)), int)
    for t_, p_ in zip(yv, pred):
        cm[t_, p_] += 1
    print('confusion matrix (rows = true, cols = predicted):', names)
    print(cm)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    export_onnx(net, a.out, len(names))
    with open(os.path.splitext(a.out)[0] + '_labels.txt', 'w') as f:
        f.write('\n'.join(names) + '\n')
    # sanity check: OpenCV DNN gives the same answers as NumPy
    cvnet = cv2.dnn.readNetFromONNX(a.out)
    same = 0
    for i in range(50):
        cvnet.setInput(Xv[i:i + 1].astype(np.float32))
        same += int(cvnet.forward().argmax() == pred[i])
    print(f'saved {a.out}  (val_acc {acc:.3f}, cv2.dnn agrees on {same}/50)')


if __name__ == '__main__':
    main()
