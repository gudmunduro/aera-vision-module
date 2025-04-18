use itertools::Itertools;
use nalgebra::{Const, SVector, Vector, Vector2};
use vision::SiftKeyPoint;
use linfa::traits::*;
use linfa_clustering::Dbscan;
use ndarray::{Array1, Array2};
use opencv::core::Mat;
use opencv::imgcodecs;
use opencv::imgcodecs::IMREAD_COLOR;
use opencv::imgproc::{cvt_color_def, COLOR_BGR2GRAY, COLOR_BGR2RGB};

const FEATURE_COUNT: usize = 40;

pub type SiftDescriptor = SVector<f32, 128>;

pub fn compare_descriptors(d1: &SiftDescriptor, d2: &SiftDescriptor) -> bool {
    (d1 - d2).norm() < 300.0
}

pub struct SiftClusterIdentifier {
    pub descriptors: Vec<SiftDescriptor>,
}

impl SiftClusterIdentifier {
    pub fn new(descriptors: Vec<SiftDescriptor>) -> Self {
        Self {
            descriptors
        }
    }

    // TODO: Merge these two methods, i.e. return None instead of feature vector when not matching
    pub fn matches_cluster(&self, cluster: &SiftCluster) -> bool {
        cluster.cluster.iter()
            .filter(|(_, d1)| self.descriptors.iter().any(|d2| compare_descriptors(d1, d2)))
            .count() > (cluster.cluster.len() / 3)
    }

    pub fn compute_feature_vector(&self, cluster: &SiftCluster) -> Vec<bool> {
        let mut features = self.descriptors
            .iter()
            .map(|d1| cluster.cluster.iter().any(|((_, d2))| compare_descriptors(d1, d2)))
            .collect_vec();

        if features.len() < FEATURE_COUNT {
            features.extend((0..(FEATURE_COUNT-features.len())).map(|_| false).collect_vec());
        }

        features
    }
}

#[derive(Clone, Debug, Default)]
pub struct SiftCluster {
    pub cluster: Vec<(Vector2<f32>, SiftDescriptor)>
}

pub struct ClassInfo {
    pub features: Vec<SiftDescriptor>,
}

pub struct SiftClusterResult {
    pub features: Vec<bool>,
    pub center: Vector2<f32>,
}

pub struct SiftProcessing {
    classes: Vec<SiftClusterIdentifier>,
}

impl SiftProcessing {
    pub fn new() -> Self {
        SiftProcessing {
            classes: Vec::new()
        }
    }

    pub fn get_feature_cluster(&mut self, sift_points: &Vec<SiftKeyPoint>) -> Vec<SiftClusterResult> {
        let clusters = cluster_points(sift_points);

        let mut results = Vec::new();
        for cluster in clusters {
            let pos: Vector2<f32> = cluster.cluster.iter().map(|(pos, _)| pos).sum::<Vector2<f32>>() / cluster.cluster.len() as f32;
            if let Some(res) = self.match_existing_cluster(pos.clone_owned(), &cluster) {
                results.push(res);
            }
            else {
                self.add_class(&cluster);
                results.push(SiftClusterResult {
                    features: (0..FEATURE_COUNT).map(|i| cluster.cluster.len() > i).collect(),
                    center: pos,
                })
            }
        }

        results
    }

    fn match_existing_cluster(&self, pos: Vector2<f32>, cluster: &SiftCluster) -> Option<SiftClusterResult> {
        let class = self.classes.iter()
            .filter(|c| c.matches_cluster(cluster))
            .next()?;

        let features = class.compute_feature_vector(&cluster);

        Some(SiftClusterResult {
            features,
            center: pos,
        })
    }

    fn add_class(&mut self, cluster: &SiftCluster) {
        let descriptors = cluster.cluster.iter().map(|(_, d)| d.clone_owned()).collect_vec();
        self.classes.push(SiftClusterIdentifier::new(descriptors))
    }
}

pub fn cluster_points(sift_points: &Vec<SiftKeyPoint>) -> Vec<SiftCluster> {
    let points: Array2<f32> = Array2::from_shape_vec((sift_points.len(), 2), sift_points.iter().flat_map(|p| vec![p.point.x as f32, p.point.y as f32]).collect_vec()).unwrap();
    let clusters: Array1<Option<usize>> = Dbscan::params(2)
        .transform(&points)
        .unwrap();

    let cluster_count = clusters.iter().map(|v| v.unwrap_or(0)).max().unwrap_or(0);
    let mut cluster_res = (0..cluster_count+1).map(|_| SiftCluster::default()).collect_vec();
    for (i, cluster_index) in clusters.iter().enumerate() {
        if let Some(cluster_index) = cluster_index {
            let pos = sift_points[i].point.clone_owned().cast::<f32>();
            let mut desc: SVector<f32, 128> = SVector::zeros();
            for di in 0..128 {
                desc.as_mut_slice()[i] = sift_points[i].feature_vec[di] as f32;
            }

            cluster_res[*cluster_index].cluster.push((pos, desc));
        }
    }

    cluster_res
}