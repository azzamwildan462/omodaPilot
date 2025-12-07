# viz_nydus_simple.py

import torch
from graphviz import Digraph

from nydus_network import make_nydus_network


def main():
    # 1) Buat model + dummy input
    model = make_nydus_network(
        num_classes=1,
        c1=32,
        c2=48,
        c3=64,
        c4=96,
        c5=128,
        c6=64,
        pools=(1, 2, 3, 6),
    )
    model.eval()

    # Dummy input (bebas, ini cuma buat ambil shape)
    x = torch.randn(1, 3, 360, 640)

    with torch.no_grad():
        size_in = tuple(x.shape)  # (B, 3, H, W)

        # Downsample branch
        x_down = model.downsample(x)         # (B, c3, H/8, W/8)

        # Global feature + PPM
        x_gfe = model.gfe_dw_pw(x_down)      # (B, c5, H/8, W/8)
        x_ppm = model.ppm(x_gfe)             # (B, c5, H/8, W/8)

        # GRU input vector (global mean)
        x_vec = x_ppm.mean(dim=[2, 3])       # (B, c5)
        seq = x_vec.unsqueeze(1)             # (B, 1, c5)
        out_seq, h_new = model.gru(seq)      # out_seq: (B, 1, c5)

        # Broadcast GRU output ke spatial
        B, T, C = out_seq.shape
        x_global_up = out_seq.view(B, C, 1, 1)
        x_global_up = torch.nn.functional.interpolate(
            x_global_up,
            size=x_down.size()[2:],
            mode="bilinear",
            align_corners=True,
        )

        # Fusion (concat)
        x_cat = torch.cat([x_down, x_global_up], dim=1)

        # Classifier + upsample
        out_logits = model.classifier(x_cat)
        out = torch.nn.functional.interpolate(
            out_logits, size=x.shape[2:], mode="bilinear", align_corners=True
        )
        size_out = tuple(out.shape)

    # 2) Bangun diagram blok dengan Graphviz
    dot = Digraph(comment="NydusNetwork (High-level)")
    dot.attr(rankdir="LR", fontsize="10")  # Left-to-right

    # Node style
    node_kwargs = dict(shape="record", style="filled", fillcolor="#e0f2ff", fontsize="10")

    # Nodes
    dot.node(
        "input",
        label=f"Input | {size_in}",
        **node_kwargs,
    )

    dot.node(
        "down",
        label=f"Learning to Downsample\\n(ConvBNReLU + 2×DW SepConv) | {tuple(x_down.shape)}",
        **node_kwargs,
    )

    dot.node(
        "gfe_ppm",
        label=(
            "Global Feature Extractor + PPM"
            "\\n(3×LinearBottleneck + PyramidPooling)"
            f" | {tuple(x_ppm.shape)}"
        ),
        **node_kwargs,
    )

    dot.node(
        "gru",
        label=(
            "GRU Head"
            "\\ninput: global mean vector"
            f" | seq: {tuple(seq.shape)}"
            f" | out_seq: {tuple(out_seq.shape)}"
        ),
        **node_kwargs,
    )

    dot.node(
        "fusion",
        label=(
            "Fusion"
            "\\nUpsample GRU output → concat dengan x_down"
            f" | x_down: {tuple(x_down.shape)}"
            f" | x_global_up: {tuple(x_global_up.shape)}"
            f" | concat: {tuple(x_cat.shape)}"
        ),
        **node_kwargs,
    )

    dot.node(
        "cls",
        label=(
            "Classifier"
            "\\n2×DW SepConv + Dropout + 1×1 Conv"
            f" | logits: {tuple(out_logits.shape)}"
        ),
        **node_kwargs,
    )

    dot.node(
        "output",
        label=f"Output seg. map | {size_out}",
        **node_kwargs,
    )

    # Edges
    dot.edge("input", "down")
    dot.edge("down", "gfe_ppm")
    dot.edge("gfe_ppm", "gru")
    dot.edge("gru", "fusion")
    dot.edge("down", "fusion", label="skip", fontsize="8", style="dashed")
    dot.edge("fusion", "cls")
    dot.edge("cls", "output")

    # 3) Render ke file PNG
    dot.format = "png"
    output_path = dot.render("nydus_simple_arch", cleanup=True)
    print(f"Saved high-level architecture to {output_path}")


if __name__ == "__main__":
    main()
