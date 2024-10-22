use super::proposal_area::ProposalArea;


#[inline(always)]
pub fn filter_too_small_regions(areas: Vec<ProposalArea>) -> Vec<ProposalArea> {
    areas.into_iter().filter(|a| (a.max - a.min).min() > 2).collect()
}