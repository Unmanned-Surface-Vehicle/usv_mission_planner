#ifndef POS_HPP
#define POS_HPP

#include<unordered_map>

struct Pos{

	double x;
	double y;
};

// Comparison method for Pos struct
bool operator==(const Pos& pos1, const Pos& pos2){
		return pos1.x == pos2.x && pos1.y == pos2.y;
}

// Comparison method for Pos struct
bool operator!=(const Pos& pos1, const Pos& pos2){
		return pos1.x != pos2.x || pos1.y != pos2.y;
}

// Comparison method for Pos struct
bool operator<(const Pos& pos1, const Pos& pos2){
		return pos1.x < pos2.x || pos1.y < pos2.y;
}

// Hash definition for specialized <unordered_set> of Pos struct
namespace std{

	template<> struct hash<Pos>{
        typedef Pos argument_type;
        typedef std::size_t result_type;
		std::size_t operator()(const Pos& p) const noexcept
		{
			int xx, yy;
			xx = (int)p.x;
			yy = (int)p.y;
		    return std::hash<int>()(xx ^ (yy << 4));
		}
	};
}

#endif