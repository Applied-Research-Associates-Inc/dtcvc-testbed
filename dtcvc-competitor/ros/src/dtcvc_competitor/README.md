# Sample DTC-VC Competitor Implementation

This is a sample DTC-VC competitor implementation which demonstrates communication with CARLA and the DTC scoring service. It randomly generates triage reports and publishes them to the configured topic for the scoring service to consume.

The sample implements PyVHR for heart rate analysis to demonstrate GPU/CUDA usage within the competitor container: https://github.com/phuselab/pyVHR

The method used for analysis is `cupy_CHROM` based on the following rPPG:
<cite>De Haan, G., & Jeanne, V. (2013). Robust pulse rate from chrominance-based rPPG. IEEE Transactions on Biomedical Engineering, 60(10), 2878-2886.</cite>