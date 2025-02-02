# -*- coding:UTF-8 -*-

import os
import sys
import torch
import datetime
import torch.utils.data
import numpy as np
import time

from tqdm import tqdm

from configs import odometry_args
from tools.excel_tools import SaveExcel
from tools.euler_tools import quat2mat, euler2quat, mat2euler
from tools.logger_tools import log_print, creat_logger
from kitti_pytorch import points_dataset
from dylo_model import dylo_model, get_loss
from utils1.collate_functions import collate_pair_wo_label


args = odometry_args()

'''CREATE DIR'''
base_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(base_dir)
experiment_dir = os.path.join(base_dir, 'experiment')
if not os.path.exists(experiment_dir): os.makedirs(experiment_dir)
if not args.task_name:
    file_dir = os.path.join(experiment_dir, '{}_KITTI_{}'.format(args.model_name, str(
        datetime.datetime.now().strftime('%Y-%m-%d_%H-%M'))))
else:
    file_dir = os.path.join(experiment_dir, args.task_name)
if not os.path.exists(file_dir): os.makedirs(file_dir)
eval_dir = os.path.join(file_dir, 'eval')
if not os.path.exists(eval_dir): os.makedirs(eval_dir)
log_dir = os.path.join(file_dir, 'logs')
if not os.path.exists(log_dir): os.makedirs(log_dir)
checkpoints_dir = os.path.join(file_dir, 'checkpoints/dylonet')
if not os.path.exists(checkpoints_dir): os.makedirs(checkpoints_dir)


'''LOG'''

def main():

    global args

    train_dir_list = [0, 1, 2, 3, 4, 5, 6]#[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    test_dir_list = [7, 8, 9, 10]

    logger = creat_logger(log_dir, args.model_name)
    logger.info('----------------------------------------TRAINING----------------------------------')
    logger.info('PARAMETER ...')
    logger.info(args)

    os.environ['CUDA_VISIBLE_DEVICES'] = '0,1,2,3'

    excel_eval = SaveExcel(test_dir_list, log_dir)
    model = dylo_model(args, args.batch_size, args.H_input, args.W_input, args.is_training)

    # train set
    train_dataset = points_dataset(
        is_training = 1,
        num_point=args.num_points,
        data_dir_list=train_dir_list,
        config=args
    )
    train_loader = torch.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        collate_fn=collate_pair_wo_label,
        pin_memory=True,
        worker_init_fn=lambda x: np.random.seed((torch.initial_seed()) % (2 ** 32))
    )#collate_fn=collate_pair,

    torch.backends.cudnn.benchmark = True
    torch.cuda.set_device(args.gpu)
    model.cuda()
    log_print(logger, 'just one gpu is:' + str(args.gpu))

    if args.optimizer == 'SGD':
        optimizer = torch.optim.SGD(model.parameters(), lr=args.learning_rate,
                                    momentum=args.momentum)
    elif args.optimizer == 'Adam':
        optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate, betas=(0.9, 0.999),
                                     eps=1e-08, weight_decay=args.weight_decay)
    optimizer.param_groups[0]['initial_lr'] = args.learning_rate
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=args.lr_stepsize,
                                                gamma=args.lr_gamma, last_epoch=-1)

    if args.ckpt is not None:
        checkpoint = torch.load(args.ckpt)
        model.load_state_dict(checkpoint['model_state_dict'])
        optimizer.load_state_dict(checkpoint['opt_state_dict'])
        scheduler.load_state_dict(checkpoint['scheduler'])
        init_epoch = checkpoint['epoch']
        log_print(logger, 'load model {}'.format(args.ckpt))

    else:
        init_epoch = 0
        log_print(logger, 'Training from scratch')


    # eval once before training
    
    if args.eval_before == 1:
        eval_pose(model, test_dir_list, init_epoch)
        # excel_eval.update(eval_dir)

    for epoch in range(init_epoch + 1, args.max_epoch):
        total_loss = 0
        total_seen = 0
        optimizer.zero_grad()

        for i, data in tqdm(enumerate(train_loader, 0), total=len(train_loader), smoothing=0.9):
            
            torch.cuda.synchronize()
            start_train_one_batch = time.time()

            pos2, pos1, sample_id, T_gt, T_trans, T_trans_inv, Tr = data
            torch.cuda.synchronize()
            pos2 = [b.cuda() for b in pos2]
            pos1 = [b.cuda() for b in pos1]
            T_trans = T_trans.cuda().to(torch.float32)
            T_trans_inv = T_trans_inv.cuda().to(torch.float32)
            T_gt = T_gt.cuda().to(torch.float32)
            model = model.train()

            torch.cuda.synchronize()
            l0_q, l0_t, l1_q, l1_t, l2_q, l2_t, l3_q, l3_t, pc1_ouput, q_gt, t_gt, w_x, w_q = model(pos2, pos1, T_gt, T_trans, T_trans_inv)
            loss = get_loss(l0_q, l0_t, l1_q, l1_t, l2_q, l2_t, l3_q, l3_t, q_gt, t_gt, w_x, w_q)
            torch.cuda.synchronize()

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            torch.cuda.synchronize()


            if args.multi_gpu is not None:
                total_loss += loss.mean().cpu().data * args.batch_size
            else:
                total_loss += loss.cpu().data * args.batch_size
            total_seen += args.batch_size

        scheduler.step()
        lr = max(optimizer.param_groups[0]['lr'], args.learning_rate_clip)
        for param_group in optimizer.param_groups:
            param_group['lr'] = lr

        train_loss = total_loss / total_seen
        log_print(logger,'EPOCH {} train mean loss: {:04f}'.format(epoch, float(train_loss)))

        if epoch % 5 == 0:
            save_path = os.path.join(checkpoints_dir,
                                     '{}_{:03d}_{:04f}.pth.tar'.format(model.__class__.__name__, epoch, float(train_loss)))
            torch.save({
                'model_state_dict': model.module.state_dict() if args.multi_gpu else model.state_dict(),
                'opt_state_dict': optimizer.state_dict(),
                'scheduler': scheduler.state_dict(),
                'epoch': epoch
            }, save_path)
            log_print(logger, 'Save {}...'.format(model.__class__.__name__))
        if epoch % 5 == 0:
            eval_pose(model, test_dir_list, epoch)
            # excel_eval.update(eval_dir)


def eval_pose(model, test_list, epoch):
    for item in test_list:
        test_dataset = points_dataset(
            is_training = 0,
            num_point = args.num_points,
            data_dir_list = [item],
            config = args
        )
        test_loader = torch.utils.data.DataLoader(
            test_dataset,
            batch_size=args.eval_batch_size,
            shuffle=False,
            num_workers=args.workers,
            collate_fn=collate_pair_wo_label,
            pin_memory=True,
            worker_init_fn=lambda x: np.random.seed((torch.initial_seed()) % (2 ** 32))
        )
        line = 0
        
        total_time = 0

        for batch_id, data in tqdm(enumerate(test_loader), total=len(test_loader), smoothing=0.9):

            torch.cuda.synchronize()
            start_prepare = time.time()
            pos2, pos1, sample_id, T_gt, T_trans, T_trans_inv, Tr = data

            torch.cuda.synchronize()
            #print('data_prepare_time: ', time.time() - start_prepare)

            pos2 = [b.cuda() for b in pos2]
            pos1 = [b.cuda() for b in pos1]
            T_trans = T_trans.cuda().to(torch.float32)
            T_trans_inv = T_trans_inv.cuda().to(torch.float32)
            T_gt = T_gt.cuda().to(torch.float32)

            model = model.eval()

            with torch.no_grad():
                
                torch.cuda.synchronize()
                start_time = time.time()

                l0_q, l0_t, l1_q, l1_t, l2_q, l2_t, l3_q, l3_t, pc1_ouput, q_gt, t_gt, w_x, w_q = model(pos2, pos1, T_gt, T_trans, T_trans_inv)

                torch.cuda.synchronize()
                #print('eval_one_time: ', time.time() - start_time)

                torch.cuda.synchronize()
                total_time += (time.time() - start_time)

                pc1_sample_2048 = pc1_ouput.cpu()
                l0_q = l3_q.cpu()
                l0_t = l3_t.cpu()
                pc1 = pc1_sample_2048.numpy()
                pred_q = l0_q.numpy()
                pred_t = l0_t.numpy()
                
                # deal with a batch_size
                for n0 in range(pc1.shape[0]):

                    cur_Tr = Tr[n0, :, :]

                    qq = pred_q[n0:n0 + 1, :]
                    qq = qq.reshape(4)
                    tt = pred_t[n0:n0 + 1, :]
                    tt = tt.reshape(3, 1)
                    RR = quat2mat(qq)
                    eular = mat2euler(RR)
                    q = euler2quat(z=eular[0])
                    RR = quat2mat(q)
                    filler = np.array([0.0, 0.0, 0.0, 1.0])
                    filler = np.expand_dims(filler, axis=0)  ##1*4

                    TT = np.concatenate([np.concatenate([RR, tt], axis=-1), filler], axis=0)

                    # TT = np.matmul(cur_Tr, TT)
                    # TT = np.matmul(TT, np.linalg.inv(cur_Tr))

                    if line == 0:
                        T_final = TT 
                        T = T_final[:3, :]
                        T = T.reshape(1, 1, 12)
                        line += 1
                    else:
                        T_final = np.matmul(T_final, TT)
                        T_current = T_final[:3, :]
                        T_current = T_current.reshape(1, 1, 12)
                        T = np.append(T, T_current, axis=0)
        
        avg_time = total_time / 4541
        #print('avg_time: ', avg_time)

        T = T.reshape(-1, 12)

        fname_file = os.path.join(log_dir, str(item).zfill(2) + '_pred.npy')
        fname_txt = os.path.join(log_dir, str(item).zfill(2) + '_pred.txt')
        data_dir = os.path.join(eval_dir, 'odometry_' + str(item).zfill(2))
        if not os.path.exists(data_dir) :
            os.makedirs(data_dir)
        np.save(fname_file, T)
        np.savetxt(fname_txt, T)
        os.system('cp %s %s' % (fname_file, data_dir))  ###SAVE THE txt FILE
        os.system('cp %s %s' % (fname_txt, data_dir))  ###SAVE THE txt FILE
        os.system('python evaluation.py --result_dir ' + data_dir + ' --eva_seqs ' + str(item).zfill(
            2) + '_pred')
    return 0


if __name__ == '__main__':
    main()
