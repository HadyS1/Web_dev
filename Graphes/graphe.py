#SOUAIBY Hady 232105
import networkx as nx
import matplotlib.pyplot as plt
import spicy as sp

#les commandes suivantes rendent un objet de type graphe full mesh d'ordre 7
cG=nx.complete_graph(7)
nx.draw(cG, with_labels=True)
plt.savefig('completeGraph.png')

G=nx.Graph()

#Questions 4, 5 et 6:
G.add_nodes_from([1,2,3,4,5,6,'A','B','C','D'])
G.add_edges_from([(1,2),(2,6),(1,6),(6,5),(2,3),(3,4),('A','B'),('A','D'),('D','B'),('C','B')])
nx.draw(G,with_labels=True)
plt.savefig('Graph1.png')

#Question 7: Essayons les fonctions G.nodes, G.edges, G.order, G.degree, G.neighbors, G.number_of_edges.
print("Les nodes de G sont: ", G.nodes())
print("Les arretes de G sont: ", G.edges())
print("L'ordre de G est: ", G.order(), G.degree())
print("Le degre de A est: ", G.degree('A'))
print("Les voisins de node 6: ", list(G.neighbors(6)))
print("Le graphe G a", G.number_of_edges(), "arretes.")

#Question 8: Assurons nous que Somme(degres) = 2*arretes
s = 0
for i in G.nodes:
    s = s + G.degree(i)
if s == 2*G.number_of_edges():
    print("La somme des degres est egal au double du nombre d'arretes: ", s)


#ou bien
#On compte le nombre de voisins pour savoir le degre: ceci ne fonctionne pas pour une loop
#m =0
#for i in G.nodes:
#    m +=len([n for n in G.neighbors(i)])
#print(m==2*G.number_of_edges()) 
#  

#Question 10
G_from_file=nx.read_edgelist('./graph.edges',nodetype=str)
#./ dans le folder courant
#notetype string car on veut les lire comme string (il y a des lettres)

#Save a graph
nx.write_edgelist(G, "graph.edges")
nx.write_graphml_lxml(G,"./graph.graphml")

#Question 11:
l=list(nx.bfs_edges(G,1))
print('BFS : ', l)
#On remarque que cette commande realise une recherche en largeur (Breadth-First Search) en commenceant par le sommet 1
#En effet, les voisins de 1 sont 2 et 6, qui font parti des premiers tuples de la liste, et ainsi de suite.

#Question 12
l=list(nx.dfs_edges(G,1))
print('DFS', l)
#On remarque que cette commande realise une recherche en profondeur (Depth-First Search) en commenceant par le sommet 1
#En effet, un des voisins de 1 est 2 qui devient l'antecedent des voisins a chercher, et ainsi de suite.

#Question 13:
def CompConnex(G):
    marque = {}
    compID = 0
    for x in G.nodes:
        marque[x] = 0
    for x in G.nodes:
        if marque[x] == 0:
            compID += 1
            ParcoursLargeur(G, x, compID, marque)
    return (marque, compID)

def ParcoursLargeur(G, s, compID, marque):
    file = []
    file.append(s)
    marque[s] = compID

    while file:
        x = file.pop(0)  #Retire le premier element (FIFO)
        for y in G.neighbors(x):
            if marque[y] == 0:
                file.append(y)
                marque[y] = compID

resultat = CompConnex(G)
marque, compID = resultat
print("Le nombre de composantes connexes: ", compID)
print("Sommets et leurs composantes: ", marque)

#Arretes de chaque composante
atteint = []
for i in G.nodes():
    if marque[i] not in atteint:
        l = list(nx.bfs_edges(G, i))
        print('Les arretes de la composante', marque[i], "sont: ", l)
        atteint.append(marque[i])

#Question 14:
M=nx.adjacency_matrix(G)
print('adjacency matrix of G: \n', M)
#Ces colonnes me rend le nombre d'arretes entre (i,j)

M=nx.attr_matrix(G,rc_order=[1,2,3,4,5,6,'A','B','C','D'])
print('adjacency matrix of G: \n', M)
#Cette fonction ajuste les coordonnes et valuers de facons a les visualiser sous la forme d'une matrice d'ajacence
#M(i,j) est le nombre d'arretes entre i et j

#transitive closure
from numpy import *
n=G.order()
I=eye(n,n)
TC=(I+M)**(n-1)
TC_b=mat(TC,dtype=bool)
#Ici, TC est la matrice qui donne nombre de chemins entre i et j (Identitie + matrice initial) exposant n-1
#TC_b est de meme binaire, c'est donc la fermeture transistive de M qui donne s'il exist une chemin entre i et j

#Question 15:
#Cette fonction est un parcours en largeur tel qur order est le niveau de visite
def functionX(G,s) :
    F=[]
    order=1
    mark={}
    for i in G.nodes() :
        mark[i]=-1
    mark[s]=order
    F.append(s)
    while F :
        x=F.pop(0)
        for y in G.neighbors(x) :
            if mark[y]==-1 : #toujours non visite
                F.append(y)
                order=order+1
                mark[y]=order
    return mark

#Question 16:
def IsCyclique(G,s) :
    GrapheCyclique = True
    F=[]
    order=1
    mark={}
    pred ={}
    for i in G.nodes() :
        mark[i] = -1
        pred[i] = s
    mark[s]= order
    F.append(s)

    while F :
        x=F.pop(0)
        for y in G.neighbors(x) :
            if mark[y]==-1 : #toujours non visite
                F.append(y)
                order=order+1
                mark[y]=order
                pred[y] =x
            else:
                if pred[x]!= y:
                    return GrapheCyclique
    GrapheCyclique = False
    return GrapheCyclique
if IsCyclique(G,1) ==True:
    print("Le graphe est cyclique")
else:
    print("Le graphe est acyclique")

#Question 17
G['B']['A']['weight']=4
G.nodes[1]['color']= 'red'
print(G.nodes(data=True))
print(G.edges(data=True))
#Ce programme donne des attributs aux arretes (poids) et aux sommets (couleur) puis affiche ces attributs
#On remarque que le dictionnaire est vide pour les arretes et sommets autre que 'B', 'A', et '1'

#Question 18:
content = """H1 1 2
H1 3 8
H2 1 7
H2 2 5
1 2 1
1 4 5
2 1 3
2 4 3
3 4 2
3 V 3
4 3 1
4 V 5
"""
with open("wG.txt", "w") as file:
    file.write(content)

wG=nx.read_edgelist('./wG.txt',create_using=nx.DiGraph(),nodetype=str,data=(('weight',int),))
#Cette commande cree un graphe oriente pondere en utilisant le texte wG.txt tel que les sommets sont des strings.
nx.draw(wG,with_labels=True)
plt.savefig('Graph2.png')
#print(wG.nodes())

#Question 19:
'''
#Si on ne veut pas utiliser le dictionnaire pred (autre methode)
def indice(chemin, s):
    if s in chemin:
        indice = 0
        for i in chemin:
            if i == s:
                return indice
            indice += 1
    else:
        print("Le sommet n'a pas d'indice car il n'est pas dans la liste")
'''

def Dijkstra(wG,s,d):
    dist = {}
    pred = {}
    #chemin = [] (methode 1)
    for i in wG.nodes():
        dist[i] = float('inf')
        #pred[i]=s
    dist[s] =0
    pred[s]=s
    nodesleft = list(wG.nodes())
    while nodesleft:
        min = nodesleft[0]
        for i in nodesleft:
            if dist[i] <dist[min]:
                min =i
        for j in wG.neighbors(min):
            if dist[j] > dist[min] + wG[min][j]['weight']:
                dist[j] = dist[min] + wG[min][j]['weight']
                pred[j] = min
        nodesleft.remove(min)
        #chemin.append(min)

    p =s
    chemin = []
    for i in pred:
        if pred[i] == p:
            chemin.append(i)
            p = i
    chemin.append(d)
    return chemin
    #return chemin[indice(chemin,s): indice(chemin,d)+1] (methode 1)

print("Le plus court chemin entre H1 et V: ",Dijkstra(wG,'H1','V'))
print("Le plus court chemin entre H2 et 2: ",Dijkstra(wG,'H2','2'))

#Question 20:
content2 = """0 1 3
0 2 10
0 3 15
4 1 10
4 2 5
4 3 9
4 5 12
4 6 11
5 2 12
5 6 13
6 1 4
"""
with open("bG.txt", "w") as file:
    file.write(content2)

bG=nx.read_edgelist('./bG.txt',create_using=nx.Graph(),nodetype=str,data=(('bandwidth',int),))

#Question 21
def minimum(a,b):
    if a<b or a ==b:
        min=a
    else:
        min = b
    return min

def Dijkstrabandwidth(bG,s,d):
    dist = {}
    pred = {}
    for i in bG.nodes():
        dist[i] = 0
        pred[i]=s
    dist[s] =float('inf')
    #pred[s]=s
    nodesleft = list(bG.nodes())
    while nodesleft:
        max = nodesleft[0]
        for i in nodesleft:
            if dist[i] > dist[max]:
                max =i
        for j in bG.neighbors(max):
            if dist[j] < minimum(dist[max], bG[max][j]['bandwidth']):
                dist[j] = minimum(dist[max], bG[max][j]['bandwidth'])
                pred[j] = max
        nodesleft.remove(max)

    p = s
    chemin = []
    for i in pred:
        if pred[i] == p:
            chemin.append(i)
            p = i
    return chemin

print(Dijkstrabandwidth(bG,'0','6'))

#Question 22:
iG=nx.read_edgelist('./internet_graph.txt',create_using=nx.Graph(),nodetype=str,data=(('weight',int),))

#Question 23:
print(len(iG.nodes()))
resultat2 = CompConnex(iG)
marque2, compID2 = resultat2
if compID2 == 1:
    print("L'internet est connecte")

#Question 24:
histDeg=nx.degree_histogram(iG)
print(histDeg)
print (len(histDeg)-1)
#nx.degree_histogram(iG) nous donne une liste ou l'element a l'indice i est le nombre de sommets de degre i
#en affichant len(histDeg) avec -1 (pour ne pas compter le degre 0), on obtient le degre maximal du graphe
#si on veut savoir combien de sommet on ce degre maximum: print(histDeg[-1])

#Question 25:
#Un graphe scale-free possede un petit nombre de nodes qui ont un degre beaucoup plus eleve que le reste des nodes.
#Il n'y a donc pas de 'scale' a suivre'
#En effet, l'internet est un scale free graph car la plupart des hubs n'ont pas beaucoup de connexion.
plt.xscale('log')
plt.bar(range(len(histDeg)),histDeg)
plt.savefig('scale_free_network.png')