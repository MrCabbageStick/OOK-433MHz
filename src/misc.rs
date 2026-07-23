pub enum ThreeStateResult<Ok, Err> {
    Ok(Ok),
    NotReady,
    Err(Err),
}
