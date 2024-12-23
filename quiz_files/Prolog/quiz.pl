color(blue).
color(green).
color(white).

bottle(b1).
bottle(b2).
fruit(f1).
cup(c1).

hasColor(b1, blue).
hasColor(b2, green).
hasColor(f1, green).
hasColor(c1, white).

blue_bottle(X):- bottle(X), hasColor(X, blue).
green_object(X):- (bottle(X); fruit(X)), hasColor(X, green).
object(X):- bottle(X); fruit(X); cup(X).
obj_hasColor(X, C):- object(X), hasColor(X, C).
