% NOTA: Verifica as "seis restrições" para matrizes ortonormadas unitárias 
% (i.e. há 9 números numa matriz 3x3, mas apenas 3 são independentes)
% Função devolve 1 em caso a Matriz seja ortonormada e 0 caso não seja.
function resultado = verifica_ortogonalidade(R)

% A cada restrição ultrapassada é incrementado 1 a uma variável denominada "aux"
aux=0;

%% 1 - Verifica se o determinante é 1
if single(det(R)) == 1
    aux = aux+1;
end 

%% 2 - Verifica se a norma das colunas é 1
for C=1:3
    if single(norm(R(:,C))) == 1
        aux=aux+1;
    end    
end

%% 3 - Verifica se a norma das linhas é 1
for L=1:3
    if single(norm(R(L,:))) == 1
        aux=aux+1;
    end    
end

%% 4 - Verifica se a tranposta é igual a inversa (R = R' = inv(R))
if single(R')==single(inv(R))
   aux=aux+1; 
end

%% 5 - Verifica se R * R' = I
if single(R * R')==single(eye(3))
   aux=aux+1; 
end

%% 6 - Verifica se R * inv(R) = I
if single(R * inv(R))==single(eye(3))
   aux=aux+1; 
end

%% Após verificar as seis restrições, concluimos se a matriz é ou não ortonormada
if aux == 8
    disp('A matriz é ortonormada!');
    resultado = 1;
else
    disp('A matriz não é ortonormada!');
    resultado = 0;
end