from controller import Supervisor

# Constantes
TIME_STEP = 512
QTDD_CAIXA = 20  # CAIXA01 a CAIXA20

def main():
    # Inicializa o supervisor
    robot = Supervisor()
    
    # Array para armazenar as referências das caixas
    caixas = []
    
    print("Carregando posições das caixas...\n")
    
    # Carrega todas as caixas
    for i in range(1, QTDD_CAIXA + 1):  # CAIXA01 a CAIXA20
        nome_caixa = f"CAIXA{i:02d}"
        caixa_node = robot.getFromDef(nome_caixa)
        
        if caixa_node:
            caixas.append({
                'name': nome_caixa,
                'node': caixa_node,
                'index': i
            })
            print(f"{i:2d}. {nome_caixa} - {hex(id(caixa_node))}")
        else:
            print(f"Falha ao carregar a posição da {nome_caixa}")
    
    print(f"\n\n{len(caixas)} CAIXAS CARREGADAS\n\n")
    
    # Loop principal - mostra as posições das caixas
    while robot.step(TIME_STEP) != -1:
        print("           X       Y       Z")
        print("-" * 35)
        
        for caixa in caixas:
            posicao = caixa['node'].getPosition()
            print(f"{caixa['name']} {posicao[0]:6.2f}  {posicao[1]:6.2f}  {posicao[2]:6.2f}")
        
        print("\n" + "="*50 + "\n")
        
        # Adiciona informações sobre massa das caixas
        print("Informações das caixas:")
        print("Nome      Massa    Material")
        print("-" * 30)
        
        for caixa in caixas:
            # Tenta obter a massa da caixa
            try:
                mass_field = caixa['node'].getField('mass')
                if mass_field:
                    massa = mass_field.getSFFloat()
                    material = "Leve" if massa < 1.0 else "Pesada"
                    print(f"{caixa['name']}   {massa:6.2f}kg  {material}")
                else:
                    print(f"{caixa['name']}   N/A      N/A")
            except:
                print(f"{caixa['name']}   N/A      N/A")
        
        print("\n" + "="*50 + "\n")
    
        import time
        time.sleep(1)

if __name__ == "__main__":
    main()