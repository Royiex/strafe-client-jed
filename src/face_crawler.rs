struct VertexId(usize);
struct EdgeId(usize);
struct FaceId(usize);

//Vertex <-> Edge <-> Face -> Collide
enum FEV{
	Face(FaceId),
	Edge(EdgeId),
	Vertex(VertexId),
}

struct State{
	time:Time,
	fev:FEV,
}

enum Transition{
	Miss,
	NextState(State),
	Hit(FaceId,Time),
}
